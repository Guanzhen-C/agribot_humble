#!/usr/bin/env python3

"""Strict, cached Ollama client for local Agribot semantic models."""

import base64
import binascii
import hashlib
import json
import math
import mimetypes
import pathlib
import socket
import sqlite3
import time
import urllib.error
import urllib.parse
import urllib.request


DEFAULT_BASE_URL = "http://172.18.80.26:11434"
DEFAULT_VISION_MODEL = "qwen3.8:27b"
DEFAULT_EMBEDDING_MODEL = "qwen3-embedding:8b"
DEFAULT_EMBEDDING_DIMENSIONS = 4096


class OllamaSemanticError(RuntimeError):
    pass


def strict_json_loads(value, description):
    def reject_constant(constant):
        raise ValueError("non-finite JSON number: {}".format(constant))

    def reject_duplicate_keys(pairs):
        result = {}
        for key, item in pairs:
            if key in result:
                raise ValueError("duplicate JSON key: {}".format(key))
            result[key] = item
        return result

    try:
        return json.loads(
            value,
            parse_constant=reject_constant,
            object_pairs_hook=reject_duplicate_keys,
        )
    except (TypeError, ValueError, json.JSONDecodeError) as error:
        raise OllamaSemanticError(
            "{} is not strict JSON: {}".format(description, error)
        ) from error


def validate_base_url(value):
    parsed = urllib.parse.urlsplit(str(value))
    if (
        parsed.scheme not in ("http", "https")
        or not parsed.hostname
        or parsed.username is not None
        or parsed.password is not None
        or parsed.query
        or parsed.fragment
        or parsed.path not in ("", "/")
    ):
        raise OllamaSemanticError(
            "Ollama base URL must be HTTP(S) without credentials, path, query or fragment"
        )
    return str(value).rstrip("/")


def image_data_url(path):
    path = pathlib.Path(path)
    mime_type = mimetypes.guess_type(path.name)[0]
    if mime_type not in ("image/jpeg", "image/png", "image/webp"):
        raise OllamaSemanticError("unsupported image type: {}".format(path))
    return "data:{};base64,{}".format(
        mime_type, base64.b64encode(path.read_bytes()).decode("ascii")
    )


def _ollama_message(content):
    if isinstance(content, str):
        return {"role": "user", "content": content}
    if not isinstance(content, list) or not content:
        raise OllamaSemanticError("Ollama chat content is invalid")
    text_parts = []
    images = []
    for part in content:
        if not isinstance(part, dict):
            raise OllamaSemanticError("Ollama chat content item is invalid")
        if part.get("type") == "text":
            text = part.get("text")
            if not isinstance(text, str) or not text.strip():
                raise OllamaSemanticError("Ollama chat text is empty")
            text_parts.append(text)
        elif part.get("type") == "image_url":
            image = part.get("image_url")
            url = image.get("url") if isinstance(image, dict) else None
            if not isinstance(url, str) or not url.startswith("data:image/") or ";base64," not in url:
                raise OllamaSemanticError("Ollama image must be a base64 data URL")
            encoded = url.split(",", 1)[1]
            try:
                base64.b64decode(encoded, validate=True)
            except (ValueError, binascii.Error) as error:
                raise OllamaSemanticError("Ollama image data is invalid") from error
            images.append(encoded)
        else:
            raise OllamaSemanticError("Ollama chat content type is unsupported")
    if not text_parts:
        raise OllamaSemanticError("Ollama chat requires text instructions")
    message = {"role": "user", "content": "\n".join(text_parts)}
    if images:
        message["images"] = images
    return message


class OllamaSemanticClient:
    def __init__(
        self,
        base_url=DEFAULT_BASE_URL,
        cache_path="~/.cache/agribot/ollama_semantic.sqlite3",
        timeout=180.0,
        maximum_retries=3,
        keep_alive="30m",
    ):
        self.base_url = validate_base_url(base_url)
        self.timeout = float(timeout)
        self.maximum_retries = int(maximum_retries)
        self.keep_alive = str(keep_alive)
        if not math.isfinite(self.timeout) or self.timeout <= 0.0:
            raise OllamaSemanticError("Ollama timeout must be positive")
        if self.maximum_retries < 1:
            raise OllamaSemanticError("maximum retries must be positive")
        self.cache_path = pathlib.Path(cache_path).expanduser().resolve()
        self.cache_path.parent.mkdir(parents=True, exist_ok=True)
        self.database = sqlite3.connect(str(self.cache_path), timeout=30.0)
        self.database.execute(
            "CREATE TABLE IF NOT EXISTS embeddings ("
            "cache_key TEXT PRIMARY KEY, model TEXT NOT NULL, dimensions INTEGER NOT NULL, "
            "text_sha256 TEXT NOT NULL, vector_json TEXT NOT NULL)"
        )
        self.database.commit()
        self.opener = urllib.request.build_opener(urllib.request.ProxyHandler({})).open

    def _request(self, endpoint, document, description):
        request = urllib.request.Request(
            self.base_url + endpoint,
            data=json.dumps(document, ensure_ascii=False).encode("utf-8"),
            headers={
                "Content-Type": "application/json",
                "Accept": "application/json",
                "User-Agent": "agribot-local-semantic/1",
            },
            method="POST",
        )
        for attempt in range(self.maximum_retries):
            try:
                with self.opener(request, timeout=self.timeout) as response:
                    return strict_json_loads(
                        response.read().decode("utf-8"), description
                    )
            except urllib.error.HTTPError as error:
                body = error.read(4096).decode("utf-8", errors="replace")
                retryable = error.code in (408, 409, 429) or error.code >= 500
                if not retryable or attempt + 1 >= self.maximum_retries:
                    raise OllamaSemanticError(
                        "{} failed with HTTP {}: {}".format(
                            description, error.code, body[:1500]
                        )
                    ) from error
            except (urllib.error.URLError, socket.timeout, TimeoutError) as error:
                if attempt + 1 >= self.maximum_retries:
                    raise OllamaSemanticError(
                        "{} request failed: {}".format(description, error)
                    ) from error
            time.sleep(min(10.0, 1.5 * (2 ** attempt)))
        raise OllamaSemanticError("{} exhausted retries".format(description))

    def chat_json(self, model, content, temperature=0.0):
        response = self._request(
            "/api/chat",
            {
                "model": str(model),
                "messages": [_ollama_message(content)],
                "format": "json",
                "stream": False,
                "think": False,
                "keep_alive": self.keep_alive,
                "options": {"temperature": float(temperature)},
            },
            "Ollama multimodal response",
        )
        message = response.get("message")
        value = message.get("content") if isinstance(message, dict) else None
        if not isinstance(value, str) or not value.strip():
            raise OllamaSemanticError("Ollama response contains no JSON content")
        usage = {
            "prompt_tokens": int(response.get("prompt_eval_count", 0)),
            "completion_tokens": int(response.get("eval_count", 0)),
        }
        usage["total_tokens"] = usage["prompt_tokens"] + usage["completion_tokens"]
        return strict_json_loads(value, "Ollama model JSON"), usage

    @staticmethod
    def _embedding_key(model, dimensions, text):
        digest = hashlib.sha256(text.encode("utf-8")).hexdigest()
        return "{}:{}:{}".format(model, dimensions, digest), digest

    def _cached_embedding(self, model, dimensions, text):
        key, _ = self._embedding_key(model, dimensions, text)
        row = self.database.execute(
            "SELECT vector_json FROM embeddings WHERE cache_key = ?", (key,)
        ).fetchone()
        if row is None:
            return None
        vector = strict_json_loads(row[0], "cached Ollama embedding")
        if (
            not isinstance(vector, list)
            or len(vector) != dimensions
            or any(
                not isinstance(value, (int, float)) or not math.isfinite(value)
                for value in vector
            )
        ):
            raise OllamaSemanticError("cached Ollama embedding is invalid")
        return [float(value) for value in vector]

    def _store_embedding(self, model, dimensions, text, vector):
        key, digest = self._embedding_key(model, dimensions, text)
        self.database.execute(
            "INSERT OR REPLACE INTO embeddings "
            "(cache_key, model, dimensions, text_sha256, vector_json) VALUES (?, ?, ?, ?, ?)",
            (
                key,
                model,
                dimensions,
                digest,
                json.dumps(vector, separators=(",", ":")),
            ),
        )
        self.database.commit()

    def embed_texts(
        self,
        texts,
        model=DEFAULT_EMBEDDING_MODEL,
        dimensions=DEFAULT_EMBEDDING_DIMENSIONS,
    ):
        model = str(model)
        dimensions = int(dimensions)
        values = [str(text).strip() for text in texts]
        if not values or any(not value for value in values):
            raise OllamaSemanticError("embedding input contains empty text")
        if not 64 <= dimensions <= 4096:
            raise OllamaSemanticError("embedding dimensions are invalid")
        by_text = {}
        for text in values:
            if text not in by_text:
                by_text[text] = self._cached_embedding(model, dimensions, text)
        missing = [text for text, vector in by_text.items() if vector is None]
        for offset in range(0, len(missing), 16):
            batch = missing[offset : offset + 16]
            response = self._request(
                "/api/embed",
                {
                    "model": model,
                    "input": batch,
                    "truncate": True,
                    "keep_alive": self.keep_alive,
                },
                "Ollama embedding response",
            )
            vectors = response.get("embeddings")
            if not isinstance(vectors, list) or len(vectors) != len(batch):
                raise OllamaSemanticError("embedding response count changed")
            for text, vector in zip(batch, vectors):
                if (
                    not isinstance(vector, list)
                    or len(vector) != dimensions
                    or any(
                        not isinstance(value, (int, float))
                        or not math.isfinite(value)
                        for value in vector
                    )
                ):
                    raise OllamaSemanticError("embedding response item is invalid")
                norm = math.sqrt(sum(float(value) ** 2 for value in vector))
                if norm <= 0.0:
                    raise OllamaSemanticError("embedding has zero norm")
                normalized = [float(value) / norm for value in vector]
                by_text[text] = normalized
                self._store_embedding(model, dimensions, text, normalized)
        return [by_text[text] for text in values]
