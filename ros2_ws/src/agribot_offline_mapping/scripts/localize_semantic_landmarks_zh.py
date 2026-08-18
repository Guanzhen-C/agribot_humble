#!/usr/bin/env python3

"""Translate stable semantic landmark text into resumable Chinese metadata."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import socket
import urllib.error
import urllib.parse
import urllib.request


DEFAULT_BASE_URL = "https://dashscope.aliyuncs.com/compatible-mode/v1"
DEFAULT_MODEL = "qwen3.7-flash"


class LandmarkLocalizationError(RuntimeError):
    pass


def file_sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def contains_chinese(text):
    return any("\u3400" <= character <= "\u9fff" for character in str(text))


def validate_https_base_url(value):
    parsed = urllib.parse.urlsplit(value)
    if (
        parsed.scheme != "https"
        or not parsed.netloc
        or parsed.username is not None
        or parsed.password is not None
        or parsed.query
        or parsed.fragment
    ):
        raise LandmarkLocalizationError(
            "Bailian base URL must be HTTPS without credentials, query or fragment"
        )
    return value.rstrip("/")


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
        raise LandmarkLocalizationError(
            "{} is not strict JSON: {}".format(description, error)
        ) from error


def translation_request(model, pairs):
    required = {
        "translations": [
            {
                "source_caption": item[0],
                "source_category": item[1],
                "caption_zh": "简洁准确的中文地标描述",
                "category_zh": "简洁中文类别",
            }
            for item in pairs
        ]
    }
    system_prompt = (
        "你是机器人语义地图地标翻译器。把每条英文caption和category翻译为简洁、自然、可检索的中文，"
        "保留颜色、材质、数量、车辆类型和空间特征，不增加原文没有的信息。caption_zh应是一个中文"
        "名词短语，category_zh应是简洁中文类别。必须逐条保留source_caption和source_category原文，"
        "顺序和数量不得改变。只输出一个严格JSON对象，只能包含translations字段，不得输出Markdown。"
    )
    return {
        "model": model,
        "messages": [
            {"role": "system", "content": system_prompt},
            {
                "role": "user",
                "content": json.dumps(
                    {"input": required["translations"], "required_output": required},
                    ensure_ascii=False,
                    separators=(",", ":"),
                ),
            },
        ],
        "response_format": {"type": "json_object"},
        "enable_thinking": False,
        "temperature": 0.0,
    }


def call_translation_batch(pairs, api_key, base_url, model, timeout=90.0):
    if not api_key:
        raise LandmarkLocalizationError(
            "environment variable DASHSCOPE_API_KEY is not configured"
        )
    request = urllib.request.Request(
        validate_https_base_url(base_url) + "/chat/completions",
        data=json.dumps(
            translation_request(model, pairs), ensure_ascii=False
        ).encode("utf-8"),
        headers={
            "Authorization": "Bearer {}".format(api_key.strip()),
            "Content-Type": "application/json",
            "Accept": "application/json",
            "User-Agent": "agribot-landmark-localization/1",
        },
        method="POST",
    )
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            payload = response.read().decode("utf-8")
    except urllib.error.HTTPError as error:
        body = error.read(2048).decode("utf-8", errors="replace")
        raise LandmarkLocalizationError(
            "translation request failed with HTTP {}: {}".format(
                error.code, body[:1000]
            )
        ) from error
    except (urllib.error.URLError, socket.timeout, TimeoutError) as error:
        raise LandmarkLocalizationError(
            "translation request failed: {}".format(error)
        ) from error
    document = strict_json_loads(payload, "Bailian response")
    choices = document.get("choices")
    if not isinstance(choices, list) or len(choices) != 1:
        raise LandmarkLocalizationError("Bailian response has invalid choices")
    message = choices[0].get("message")
    content = message.get("content") if isinstance(message, dict) else None
    translated = strict_json_loads(content, "Bailian translation content")
    return translated


def validate_translations(document, expected_pairs):
    if not isinstance(document, dict) or set(document) != {"translations"}:
        raise LandmarkLocalizationError(
            "translation output must contain only translations"
        )
    items = document["translations"]
    if not isinstance(items, list) or len(items) != len(expected_pairs):
        raise LandmarkLocalizationError("translation output count changed")
    validated = []
    for item, expected in zip(items, expected_pairs):
        if not isinstance(item, dict) or set(item) != {
            "source_caption",
            "source_category",
            "caption_zh",
            "category_zh",
        }:
            raise LandmarkLocalizationError("translation item fields are invalid")
        pair = (str(item["source_caption"]), str(item["source_category"]))
        if pair != expected:
            raise LandmarkLocalizationError("translation source text or order changed")
        caption = str(item["caption_zh"]).strip()
        category = str(item["category_zh"]).strip()
        if not contains_chinese(caption) or not contains_chinese(category):
            raise LandmarkLocalizationError("translation output must be Chinese")
        validated.append(
            {
                "source_caption": pair[0],
                "source_category": pair[1],
                "caption_zh": caption,
                "category_zh": category,
            }
        )
    return validated


def load_existing(path, semantic_digest):
    if not path.is_file():
        return []
    document = strict_json_loads(
        path.read_text(encoding="utf-8"), "existing Chinese localization"
    )
    if document.get("schema_version") != 1:
        raise LandmarkLocalizationError("unsupported localization schema")
    if document.get("semantic_metadata_sha256") != semantic_digest:
        raise LandmarkLocalizationError(
            "existing localization belongs to different semantic metadata"
        )
    items = document.get("translations")
    if not isinstance(items, list):
        raise LandmarkLocalizationError("existing localization is invalid")
    return validate_translations(
        {"translations": items},
        [
            (str(item.get("source_caption", "")), str(item.get("source_category", "")))
            for item in items
        ],
    )


def atomic_write(path, semantic_path, semantic_digest, translations):
    document = {
        "schema_version": 1,
        "language": "zh-CN",
        "semantic_metadata": str(semantic_path),
        "semantic_metadata_sha256": semantic_digest,
        "translations": translations,
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(document, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    temporary.replace(path)


def collect_pairs(document, minimum_detections, excluded_categories):
    objects = document.get("objects")
    if not isinstance(objects, list):
        raise LandmarkLocalizationError(
            "semantic metadata must contain an objects list"
        )
    pairs = []
    seen = set()
    for item in objects:
        category = str(item.get("legacy_semantickitti_tag", "unknown"))
        if category in excluded_categories:
            continue
        if int(item.get("num_detections", 0)) < minimum_detections:
            continue
        pair = (str(item.get("caption", "object")), category)
        if pair not in seen:
            seen.add(pair)
            pairs.append(pair)
    return pairs


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--semantic-metadata", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--minimum-detections", type=int, default=10)
    parser.add_argument(
        "--excluded-categories", nargs="+", default=["road", "parking"]
    )
    parser.add_argument("--batch-size", type=int, default=30)
    parser.add_argument("--model", default=DEFAULT_MODEL)
    parser.add_argument(
        "--base-url",
        default=os.environ.get("DASHSCOPE_BASE_URL", DEFAULT_BASE_URL),
    )
    parser.add_argument("--timeout", type=float, default=90.0)
    return parser.parse_args()


def main():
    arguments = parse_args()
    if arguments.minimum_detections < 1:
        raise LandmarkLocalizationError("minimum detections must be positive")
    if not 1 <= arguments.batch_size <= 50:
        raise LandmarkLocalizationError("batch size must be between 1 and 50")
    semantic_path = arguments.semantic_metadata.expanduser().resolve()
    output = arguments.output.expanduser().resolve()
    semantic_digest = file_sha256(semantic_path)
    semantic_document = strict_json_loads(
        semantic_path.read_text(encoding="utf-8"), "semantic metadata"
    )
    requested = collect_pairs(
        semantic_document,
        arguments.minimum_detections,
        set(arguments.excluded_categories),
    )
    existing = load_existing(output, semantic_digest)
    lookup = {
        (item["source_caption"], item["source_category"]): item
        for item in existing
    }
    missing = [pair for pair in requested if pair not in lookup]
    api_key = os.environ.get("DASHSCOPE_API_KEY", "")
    for offset in range(0, len(missing), arguments.batch_size):
        batch = missing[offset:offset + arguments.batch_size]
        response = call_translation_batch(
            batch,
            api_key,
            arguments.base_url,
            arguments.model,
            arguments.timeout,
        )
        for item in validate_translations(response, batch):
            lookup[(item["source_caption"], item["source_category"])] = item
        ordered = [lookup[pair] for pair in requested if pair in lookup]
        atomic_write(output, semantic_path, semantic_digest, ordered)
        print("Localized {}/{} unique landmark descriptions.".format(
            len(ordered), len(requested)
        ))
    if not missing:
        atomic_write(output, semantic_path, semantic_digest, [lookup[p] for p in requested])
    print("Saved {} Chinese landmark translations to {}".format(len(requested), output))


if __name__ == "__main__":
    try:
        main()
    except (
        LandmarkLocalizationError,
        OSError,
        UnicodeDecodeError,
        json.JSONDecodeError,
    ) as error:
        raise SystemExit("error: {}".format(error)) from error
