"""Read the pinned Harvester geometry file with unquoted XML attributes."""

import hashlib
from html.parser import HTMLParser
from pathlib import Path
import xml.etree.ElementTree as ET


HARVESTER_SDF_SHA256 = "349e7bb922d75f541913a1981b1878f4f2cda1d1f1c78bda67405d697c8ec9c2"


class _LegacyGeometryParser(HTMLParser):
    def __init__(self):
        super().__init__(convert_charrefs=True)
        self.builder = ET.TreeBuilder()
        self.stack = []

    def handle_starttag(self, tag, attrs):
        if any(value is None for _, value in attrs):
            raise ValueError("Missing legacy SDF attribute value")
        self.builder.start(tag, dict(attrs))
        self.stack.append(tag)

    def handle_endtag(self, tag):
        if not self.stack or self.stack.pop() != tag:
            raise ValueError("Unbalanced legacy SDF element")
        self.builder.end(tag)

    def handle_startendtag(self, tag, attrs):
        self.handle_starttag(tag, attrs)
        self.handle_endtag(tag)

    def handle_data(self, data):
        if self.stack:
            self.builder.data(data)


def read_harvester_geometry(path):
    data = Path(path).read_bytes()
    if hashlib.sha256(data).hexdigest() != HARVESTER_SDF_SHA256:
        raise ValueError("Review the changed upstream SDF before importing it")
    parser = _LegacyGeometryParser()
    parser.feed(data.decode("utf-8"))
    parser.close()
    if parser.stack:
        raise ValueError("Unclosed legacy SDF element")
    root = parser.builder.close()
    if root.tag != "sdf" or len(root.findall("model")) != 1:
        raise ValueError("Expected one upstream geometry model")
    return root.find("model")
