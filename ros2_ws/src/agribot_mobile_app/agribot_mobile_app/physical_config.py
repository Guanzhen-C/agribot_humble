"""Optional imported preset; absence leaves the existing launch command intact."""

from __future__ import annotations

import json
from pathlib import Path

from agribot_vehicle_config_tools.physical_config import (
    ALGORITHMS, MAX_BYTES, PRESET, PhysicalConfigError,
    export_document, template_paths, validate_document, write_document,
)


class PhysicalConfigStore:
    def __init__(self, root: Path, paths: dict[str, Path] | None = None):
        self.root = root
        self.paths = paths
        self.document = None
        self.error = ""
        path = root / "imported.json"
        if path.is_file():
            try:
                if path.stat().st_size > MAX_BYTES:
                    raise PhysicalConfigError("真机配置文件过大")
                self.document = validate_document(
                    json.loads(path.read_text(encoding="utf-8")), self.baseline()
                )
            except (ValueError, OSError) as error:
                self.error = f"导入配置已失效，使用程序默认配置: {error}"

    def baseline(self) -> dict:
        return export_document(self.paths or template_paths())

    def public(self) -> dict:
        return {
            "source": "imported" if self.document else "default",
            "vehicle_type": "ackermann",
            "preset": PRESET,
            "algorithms": dict(ALGORITHMS),
            "sha256": self.document["sha256"] if self.document else None,
            "error": self.error,
        }

    def import_document(self, document: object) -> dict:
        validated = validate_document(document, self.baseline())
        write_document(self.root / "imported.json", validated)
        self.document = validated
        self.error = ""
        return self.public()

    def reset(self) -> dict:
        (self.root / "imported.json").unlink(missing_ok=True)
        self.document = None
        self.error = ""
        return self.public()

    def launch_arguments(self, vehicle_type: str) -> dict[str, str]:
        if self.document is None or vehicle_type != "ackermann":
            return {}
        validate_document(self.document, self.baseline())
        directory = self.root / self.document["sha256"]
        directory.mkdir(parents=True, exist_ok=True)
        arguments = {}
        for key, content in self.document["parameters"].items():
            path = directory / f"{key}.yaml"
            # Keys and contents were checked against the locally installed preset.
            path.write_text(content, encoding="utf-8")
            arguments[key] = str(path.resolve())
        return arguments
