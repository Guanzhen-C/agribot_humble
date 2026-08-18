#!/usr/bin/env python3

"""Assign audited Chinese semantics to OpenGraph 3D instances with local Ollama."""

import argparse
from functools import lru_cache
import gzip
import hashlib
import json
import math
from pathlib import Path
import pickle
import re

import cv2
import numpy as np

from ollama_semantic_client import (
    OllamaSemanticClient,
    OllamaSemanticError,
    DEFAULT_BASE_URL,
    DEFAULT_EMBEDDING_DIMENSIONS,
    DEFAULT_EMBEDDING_MODEL,
    DEFAULT_VISION_MODEL,
    image_data_url,
    strict_json_loads,
)


class InstanceDescriptionError(RuntimeError):
    pass


def file_sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def atomic_write(path, document):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(document, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    temporary.replace(path)


def load_source(pickle_path, metadata_path):
    with gzip.open(pickle_path, "rb") as stream:
        result = pickle.load(stream)
    objects = result.get("objects") if isinstance(result, dict) else None
    if not isinstance(objects, list):
        raise InstanceDescriptionError("OpenGraph pickle has no objects list")
    metadata = strict_json_loads(
        metadata_path.read_text(encoding="utf-8"), "OpenGraph semantic metadata"
    )
    entries = metadata.get("objects") if isinstance(metadata, dict) else None
    if not isinstance(entries, list) or len(entries) != len(objects):
        raise InstanceDescriptionError("OpenGraph pickle and metadata object counts differ")
    for index, (item, entry) in enumerate(zip(objects, entries)):
        if entry.get("id") != index:
            raise InstanceDescriptionError("OpenGraph object identifiers are not contiguous")
        points = np.asarray(item.get("pcd_np"), dtype=np.float64)
        if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
            raise InstanceDescriptionError("OpenGraph object point cloud is invalid")
        if int(entry.get("point_count", -1)) != len(points):
            raise InstanceDescriptionError("OpenGraph object point count changed")
    return objects, metadata


def load_poses(path):
    values = np.loadtxt(path, dtype=np.float64)
    if values.ndim == 1:
        values = values.reshape(1, -1)
    if values.ndim != 2 or values.shape[1] != 12 or len(values) < 1:
        raise InstanceDescriptionError("poses.txt must contain 12 values per row")
    transforms = np.tile(np.eye(4, dtype=np.float64), (len(values), 1, 1))
    transforms[:, :3, :4] = values.reshape(-1, 3, 4)
    if not np.isfinite(transforms).all():
        raise InstanceDescriptionError("poses.txt contains non-finite values")
    return transforms


def load_projection(path):
    rows = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        if ":" not in line:
            continue
        key, value = line.split(":", 1)
        rows[key.strip()] = [float(item) for item in value.split()]
    values = rows.get("P2")
    if values is None or len(values) != 12:
        raise InstanceDescriptionError("calib.txt has no valid P2 projection")
    projection = np.asarray(values, dtype=np.float64).reshape(3, 4)
    if not np.isfinite(projection).all() or projection[2, 2] == 0.0:
        raise InstanceDescriptionError("camera projection is invalid")
    return projection


def spread_candidates(values, maximum):
    values = sorted(set(int(value) for value in values))
    if len(values) <= maximum:
        return values
    indices = np.linspace(0, len(values) - 1, maximum).round().astype(int)
    return [values[index] for index in sorted(set(indices.tolist()))]


def sample_points(points, maximum=3000):
    if len(points) <= maximum:
        return points
    indices = np.linspace(0, len(points) - 1, maximum).round().astype(int)
    return points[indices]


def project_points(points_map, map_from_camera, projection, width, height):
    camera_points = (map_from_camera[:3, :3].T @ (
        points_map - map_from_camera[:3, 3]
    ).T).T
    camera_points = camera_points[camera_points[:, 2] > 0.20]
    if len(camera_points) < 4:
        return None
    homogeneous = np.column_stack((camera_points, np.ones(len(camera_points))))
    pixels = (projection @ homogeneous.T).T
    pixels = pixels[:, :2] / pixels[:, 2:3]
    finite = np.isfinite(pixels).all(axis=1)
    pixels = pixels[finite]
    visible = (
        (pixels[:, 0] >= 0.0)
        & (pixels[:, 0] < width)
        & (pixels[:, 1] >= 0.0)
        & (pixels[:, 1] < height)
    )
    pixels = pixels[visible]
    if len(pixels) < 4:
        return None
    return pixels


@lru_cache(maxsize=48)
def load_masks(path_text):
    path = Path(path_text)
    if not path.is_file():
        return []
    with gzip.open(path, "rb") as stream:
        observations = pickle.load(stream)
    masks = []
    for item in observations if isinstance(observations, list) else []:
        mask = np.asarray(item.get("mask")).squeeze().astype(bool)
        if mask.ndim == 2:
            masks.append(mask)
    return masks


def select_mask(pixels, masks):
    if not masks:
        return None, 0, 0.0
    rows = np.clip(np.rint(pixels[:, 1]).astype(int), 0, masks[0].shape[0] - 1)
    columns = np.clip(np.rint(pixels[:, 0]).astype(int), 0, masks[0].shape[1] - 1)
    best = None
    best_count = 0
    for mask in masks:
        if mask.shape != masks[0].shape:
            continue
        count = int(mask[rows, columns].sum())
        if count > best_count:
            best = mask
            best_count = count
    return best, best_count, best_count / max(1, len(pixels))


def crop_from_observation(image, pixels, mask, output_path):
    height, width = image.shape[:2]
    if mask is not None and mask.any():
        rows, columns = np.nonzero(mask)
        x0, x1 = int(columns.min()), int(columns.max()) + 1
        y0, y1 = int(rows.min()), int(rows.max()) + 1
    else:
        lower = np.quantile(pixels, 0.02, axis=0)
        upper = np.quantile(pixels, 0.98, axis=0)
        x0, y0 = np.floor(lower).astype(int)
        x1, y1 = np.ceil(upper).astype(int) + 1
    object_width = max(1, x1 - x0)
    object_height = max(1, y1 - y0)
    padding = max(12, int(round(0.25 * max(object_width, object_height))))
    crop_x0, crop_y0 = max(0, x0 - padding), max(0, y0 - padding)
    crop_x1, crop_y1 = min(width, x1 + padding), min(height, y1 + padding)
    if crop_x1 - crop_x0 < 12 or crop_y1 - crop_y0 < 12:
        return None
    crop = image[crop_y0:crop_y1, crop_x0:crop_x1].copy()
    if mask is not None:
        local_mask = mask[crop_y0:crop_y1, crop_x0:crop_x1]
        muted = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
        muted = cv2.cvtColor(muted, cv2.COLOR_GRAY2BGR)
        muted = cv2.addWeighted(muted, 0.65, np.full_like(muted, 235), 0.35, 0.0)
        crop[~local_mask] = muted[~local_mask]
    cv2.rectangle(
        crop,
        (max(0, x0 - crop_x0), max(0, y0 - crop_y0)),
        (min(crop.shape[1] - 1, x1 - crop_x0), min(crop.shape[0] - 1, y1 - crop_y0)),
        (0, 0, 255),
        2,
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(output_path), crop, [cv2.IMWRITE_JPEG_QUALITY, 91]):
        raise InstanceDescriptionError("failed to save instance crop")
    return {
        "crop_bbox": [int(crop_x0), int(crop_y0), int(crop_x1), int(crop_y1)],
        "target_bbox": [int(x0), int(y0), int(x1), int(y1)],
        "crop_width": int(crop.shape[1]),
        "crop_height": int(crop.shape[0]),
    }


def create_object_views(
    object_id,
    item,
    dataset_root,
    caption_directory,
    poses,
    projection,
    crop_root,
    stride,
    start,
    maximum_candidate_frames,
    maximum_views,
):
    points = sample_points(np.asarray(item["pcd_np"], dtype=np.float64))
    candidates = spread_candidates(item.get("image_idx", []), maximum_candidate_frames)
    scored = []
    for dataset_index in candidates:
        raw_index = start + dataset_index * stride
        if not 0 <= raw_index < len(poses):
            continue
        image_path = dataset_root / "image_2" / "{:06d}.png".format(raw_index)
        if not image_path.is_file():
            continue
        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if image is None or min(image.shape[:2]) < 12:
            continue
        pixels = project_points(
            points, poses[raw_index], projection, image.shape[1], image.shape[0]
        )
        if pixels is None:
            continue
        caption_path = caption_directory / "cap_{:06d}.pkl.gz".format(raw_index)
        mask, overlap_count, overlap_ratio = select_mask(
            pixels, load_masks(str(caption_path))
        )
        if mask is not None and overlap_count < 3:
            mask = None
        crop_path = (
            crop_root
            / "object_{:04d}".format(object_id)
            / "frame_{:06d}.jpg".format(raw_index)
        )
        crop_metadata = crop_from_observation(image, pixels, mask, crop_path)
        if crop_metadata is None:
            continue
        area = crop_metadata["target_bbox"][2] - crop_metadata["target_bbox"][0]
        area *= crop_metadata["target_bbox"][3] - crop_metadata["target_bbox"][1]
        score = (
            math.log1p(len(pixels))
            + 2.0 * overlap_ratio
            + 0.25 * math.log1p(max(1, area))
        )
        scored.append(
            {
                "dataset_index": dataset_index,
                "frame_index": raw_index,
                "image": str(image_path.resolve()),
                "crop": str(crop_path.resolve()),
                "crop_sha256": file_sha256(crop_path),
                "projected_points": int(len(pixels)),
                "mask_overlap_points": int(overlap_count),
                "mask_overlap_ratio": float(overlap_ratio),
                "score": float(score),
                **crop_metadata,
            }
        )
    selected = []
    for view in sorted(scored, key=lambda value: (-value["score"], value["frame_index"])):
        if any(abs(view["frame_index"] - item["frame_index"]) < stride for item in selected):
            continue
        selected.append(view)
        if len(selected) >= maximum_views:
            break
    return sorted(selected, key=lambda value: value["frame_index"])


def contains_chinese(value):
    return re.search(r"[\u3400-\u9fff]", str(value)) is not None


MOVABLE_LANDMARK_CATEGORY_TERMS = (
    "人员",
    "行人",
    "人群",
    "动物",
    "汽车",
    "轿车",
    "越野车",
    "车辆",
    "摩托车",
    "电动车",
    "自行车",
    "三轮车",
    "卡车",
    "货车",
    "巴士",
    "公交车",
    "非机动车停放区",
)


def movable_landmark_category(category):
    value = re.sub(r"\s+", "", str(category))
    return any(term in value for term in MOVABLE_LANDMARK_CATEGORY_TERMS)


def apply_local_landmark_policy(description):
    result = dict(description)
    if result.get("landmark_usable") and movable_landmark_category(
        result.get("category_zh", "")
    ):
        result["landmark_usable"] = False
        result["rejection_reason"] = "该类别属于可移动目标，不可作为长期导航地标"
    return result


def build_request(model, records):
    required = {
        "objects": [
            {
                "object_id": record["object_id"],
                "caption_zh": "只描述红框内目标的简洁中文名词短语",
                "category_zh": "简洁中文类别",
                "landmark_usable": True,
                "is_static": True,
                "is_drivable_surface": False,
                "confidence": 0.95,
                "visible_evidence": ["图中可直接复查的依据"],
                "rejection_reason": "",
            }
            for record in records
        ]
    }
    content = [
        {
            "type": "text",
            "text": (
                "你是农业移动机器人三维语义实例审计器。每组图片是同一个OpenGraph三维实例的"
                "多视角裁剪，红框及彩色区域是目标，灰白区域只是上下文。只描述目标本身，不要"
                "描述整幅场景，也不要根据常识补充不可见属性。caption_zh必须保留可见颜色、材质、"
                "数量和固定结构等辨识特征；category_zh必须是准确中文类别。固定建筑、门、标牌、"
                "围栏、立柱、树木和长期设施可作为地标；人、动物、车辆、可移动物、阴影、反光、"
                "天空、噪点和无法确认的目标不得作为地标。道路、地面、停车区等可行驶表面将"
                "is_drivable_surface设为true，但landmark_usable必须为false。只有静态、清晰、"
                "显著且适合长期自然语言导航的目标才能令landmark_usable为true。若多视角矛盾，"
                "降低confidence并拒绝作为地标。必须只输出一个严格JSON对象，不得使用Markdown，"
                "不得输出坐标、路径、控制指令或输入中没有的object_id。对象顺序不得改变。"
                "所需JSON结构：{}"
            ).format(json.dumps(required, ensure_ascii=False, separators=(",", ":"))),
        }
    ]
    for record in records:
        content.append(
            {
                "type": "text",
                "text": "object_id={}，以下{}张图片均为该目标：".format(
                    record["object_id"], len(record["views"])
                ),
            }
        )
        for view in record["views"]:
            content.append(
                {
                    "type": "image_url",
                    "image_url": {"url": image_data_url(view["crop"])},
                    "min_pixels": 65536,
                    "max_pixels": 307200,
                }
            )
    return model, content


def validate_response(document, records, minimum_confidence):
    if not isinstance(document, dict) or set(document) != {"objects"}:
        raise InstanceDescriptionError("model output must contain only objects")
    values = document["objects"]
    if not isinstance(values, list) or len(values) != len(records):
        raise InstanceDescriptionError("model changed the object count")
    expected_fields = {
        "object_id",
        "caption_zh",
        "category_zh",
        "landmark_usable",
        "is_static",
        "is_drivable_surface",
        "confidence",
        "visible_evidence",
        "rejection_reason",
    }
    result = []
    for record, item in zip(records, values):
        if not isinstance(item, dict) or set(item) != expected_fields:
            raise InstanceDescriptionError("model changed semantic output fields")
        if item["object_id"] != record["object_id"]:
            raise InstanceDescriptionError("model changed object order or identifier")
        caption = str(item["caption_zh"]).strip()
        category = str(item["category_zh"]).strip()
        if (
            not contains_chinese(caption)
            or not contains_chinese(category)
            or len(caption) > 100
            or len(category) > 30
        ):
            raise InstanceDescriptionError("model did not return valid Chinese semantics")
        booleans = [item[key] for key in (
            "landmark_usable", "is_static", "is_drivable_surface"
        )]
        if any(not isinstance(value, bool) for value in booleans):
            raise InstanceDescriptionError("model semantic flags are invalid")
        confidence = item["confidence"]
        if (
            not isinstance(confidence, (int, float))
            or not math.isfinite(confidence)
            or not 0.0 <= confidence <= 1.0
        ):
            raise InstanceDescriptionError("model confidence is invalid")
        evidence = item["visible_evidence"]
        if (
            not isinstance(evidence, list)
            or len(evidence) > 6
            or any(not isinstance(value, str) or not value.strip() or len(value) > 100 for value in evidence)
        ):
            raise InstanceDescriptionError("model visible evidence is invalid")
        reason = str(item["rejection_reason"]).strip()
        if len(reason) > 160:
            raise InstanceDescriptionError("model rejection reason is too long")
        requested = bool(item["landmark_usable"])
        promoted = (
            requested
            and bool(item["is_static"])
            and not bool(item["is_drivable_surface"])
            and float(confidence) >= minimum_confidence
            and bool(evidence)
        )
        result.append(
            apply_local_landmark_policy(
                {
                "object_id": record["object_id"],
                "caption_zh": caption,
                "category_zh": category,
                "model_landmark_usable": requested,
                "landmark_usable": promoted,
                "is_static": bool(item["is_static"]),
                "is_drivable_surface": bool(item["is_drivable_surface"]),
                "confidence": float(confidence),
                "visible_evidence": [str(value).strip() for value in evidence],
                "rejection_reason": reason,
                }
            )
        )
    return result


def describe_records(
    client,
    model,
    records,
    minimum_confidence,
    validation_retries,
):
    last_error = None
    for attempt in range(validation_retries):
        try:
            _model, content = build_request(model, records)
            response, _usage = client.chat_json(_model, content)
            return validate_response(response, records, minimum_confidence)
        except (OllamaSemanticError, InstanceDescriptionError) as error:
            last_error = error
            print(
                "Semantic batch validation attempt {}/{} failed: {}".format(
                    attempt + 1, validation_retries, error
                ),
                flush=True,
            )
    if len(records) > 1:
        print(
            "Splitting a repeatedly invalid batch of {} instances.".format(
                len(records)
            ),
            flush=True,
        )
        result = []
        for record in records:
            result.extend(
                describe_records(
                    client,
                    model,
                    [record],
                    minimum_confidence,
                    validation_retries,
                )
            )
        return result
    object_id = int(records[0]["object_id"])
    reason = "本地Ollama多模态输出连续{}次未通过严格校验：{}".format(
        validation_retries, last_error
    )
    print(
        "Rejecting instance {} after repeated invalid model responses; "
        "it will not become a navigation landmark.".format(object_id),
        flush=True,
    )
    return [
        {
            "object_id": object_id,
            "caption_zh": "未获得可靠视觉语义",
            "category_zh": "未知物体",
            "model_landmark_usable": False,
            "landmark_usable": False,
            "is_static": False,
            "is_drivable_surface": False,
            "confidence": 0.0,
            "visible_evidence": [],
            "rejection_reason": reason,
            "semantic_status": "model_response_rejected",
        }
    ]


def load_checkpoint(path, identity):
    if not path.is_file():
        return {}
    document = strict_json_loads(path.read_text(encoding="utf-8"), "semantic checkpoint")
    if document.get("identity") != identity or document.get("schema_version") != 1:
        raise InstanceDescriptionError("semantic checkpoint belongs to another run")
    descriptions = document.get("descriptions")
    if not isinstance(descriptions, dict):
        raise InstanceDescriptionError("semantic checkpoint is invalid")
    return descriptions


def save_checkpoint(path, identity, descriptions):
    atomic_write(
        path,
        {
            "schema_version": 1,
            "identity": identity,
            "descriptions": descriptions,
        },
    )


def build_output(
    metadata,
    descriptions,
    view_records,
    pickle_path,
    metadata_path,
    dataset_root,
    model,
    embedding_model,
    embedding_dimensions,
    minimum_confidence,
    embeddings,
):
    output_objects = []
    landmark_count = 0
    described_count = 0
    rejected_model_response_count = 0
    for source in metadata["objects"]:
        object_id = int(source["id"])
        description = descriptions.get(str(object_id))
        item = dict(source)
        item["source_caption"] = str(source.get("caption", ""))
        item["source_category"] = str(source.get("legacy_semantickitti_tag", ""))
        item["semantic_views"] = view_records.get(str(object_id), [])
        if description is None:
            item.update(
                {
                    "caption": "未获得可靠视觉语义",
                    "caption_zh": "未获得可靠视觉语义",
                    "category_zh": "未知物体",
                    "model_landmark_usable": False,
                    "landmark_usable": False,
                    "is_static": False,
                    "is_drivable_surface": False,
                    "semantic_confidence": 0.0,
                    "visible_evidence": [],
                    "rejection_reason": "没有可用的对齐实例裁剪",
                    "semantic_status": "no_aligned_instance_crop",
                    "semantic_source": model,
                }
            )
        else:
            description = apply_local_landmark_policy(description)
            semantic_status = description.get("semantic_status", "described")
            if semantic_status == "described":
                described_count += 1
            elif semantic_status == "model_response_rejected":
                rejected_model_response_count += 1
            item.update(
                {
                    "caption": description["caption_zh"],
                    "caption_zh": description["caption_zh"],
                    "category_zh": description["category_zh"],
                    "model_landmark_usable": description["model_landmark_usable"],
                    "landmark_usable": description["landmark_usable"],
                    "is_static": description["is_static"],
                    "is_drivable_surface": description["is_drivable_surface"],
                    "semantic_confidence": description["confidence"],
                    "visible_evidence": description["visible_evidence"],
                    "rejection_reason": description["rejection_reason"],
                    "semantic_status": semantic_status,
                    "semantic_source": model,
                }
            )
            if description["landmark_usable"]:
                landmark_count += 1
                vector = embeddings[str(object_id)]
                search_text = "{}；类别：{}".format(
                    description["caption_zh"], description["category_zh"]
                )
                item["semantic_embedding"] = {
                    "provider": "ollama_local",
                    "model": embedding_model,
                    "dimensions": embedding_dimensions,
                    "text_sha256": hashlib.sha256(search_text.encode("utf-8")).hexdigest(),
                    "vector": vector,
                }
        output_objects.append(item)
    return {
        "schema_version": 2,
        "frame_id": str(metadata.get("frame_id", "map")),
        "language": "zh-CN",
        "semantic_source": {
            "provider": "ollama_local",
            "vision_model": model,
            "embedding_model": embedding_model,
            "embedding_dimensions": embedding_dimensions,
            "minimum_landmark_confidence": minimum_confidence,
            "instance_geometry": "OpenGraph",
            "source_pickle": str(pickle_path),
            "source_metadata": str(metadata_path),
            "dataset_root": str(dataset_root),
            "sha256": {
                "source_pickle": file_sha256(pickle_path),
                "source_metadata": file_sha256(metadata_path),
                "poses": file_sha256(dataset_root / "poses.txt"),
                "calibration": file_sha256(dataset_root / "calib.txt"),
            },
        },
        "object_count": len(output_objects),
        "described_object_count": described_count,
        "rejected_model_response_count": rejected_model_response_count,
        "landmark_count": landmark_count,
        "objects": output_objects,
    }


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--opengraph-pickle", required=True, type=Path)
    parser.add_argument("--semantic-metadata", required=True, type=Path)
    parser.add_argument("--dataset-root", required=True, type=Path)
    parser.add_argument("--caption-directory", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--work-directory", required=True, type=Path)
    parser.add_argument("--model", default=DEFAULT_VISION_MODEL)
    parser.add_argument("--embedding-model", default=DEFAULT_EMBEDDING_MODEL)
    parser.add_argument(
        "--embedding-dimensions", type=int, default=DEFAULT_EMBEDDING_DIMENSIONS
    )
    parser.add_argument("--base-url", default=DEFAULT_BASE_URL)
    parser.add_argument("--cache", type=Path, default=Path("~/.cache/agribot/ollama_semantic.sqlite3"))
    parser.add_argument("--stride", type=int, default=10)
    parser.add_argument("--start", type=int, default=0)
    parser.add_argument("--maximum-candidate-frames", type=int, default=36)
    parser.add_argument("--maximum-views", type=int, default=2)
    parser.add_argument("--batch-size", type=int, default=4)
    parser.add_argument("--minimum-landmark-confidence", type=float, default=0.78)
    parser.add_argument("--timeout", type=float, default=120.0)
    parser.add_argument("--validation-retries", type=int, default=3)
    return parser.parse_args()


def main():
    arguments = parse_args()
    if arguments.stride < 1 or arguments.start < 0:
        raise InstanceDescriptionError("dataset stride and start are invalid")
    if not 1 <= arguments.maximum_views <= 4:
        raise InstanceDescriptionError("maximum views must be between one and four")
    if not 1 <= arguments.batch_size <= 4:
        raise InstanceDescriptionError("batch size must be between one and four")
    if not 4 <= arguments.maximum_candidate_frames <= 100:
        raise InstanceDescriptionError("maximum candidate frames must be between 4 and 100")
    if not 0.0 <= arguments.minimum_landmark_confidence <= 1.0:
        raise InstanceDescriptionError("minimum landmark confidence is invalid")
    if not 1 <= arguments.validation_retries <= 6:
        raise InstanceDescriptionError("validation retries must be between one and six")

    pickle_path = arguments.opengraph_pickle.expanduser().resolve()
    metadata_path = arguments.semantic_metadata.expanduser().resolve()
    dataset_root = arguments.dataset_root.expanduser().resolve()
    caption_directory = arguments.caption_directory.expanduser().resolve()
    work_directory = arguments.work_directory.expanduser().resolve()
    output_path = arguments.output.expanduser().resolve()
    for path in (pickle_path, metadata_path, dataset_root / "poses.txt", dataset_root / "calib.txt"):
        if not path.exists():
            raise InstanceDescriptionError("required input does not exist: {}".format(path))
    work_directory.mkdir(parents=True, exist_ok=True)
    crop_root = work_directory / "instance_crops"
    checkpoint_path = work_directory / "semantic_checkpoint.json"

    objects, metadata = load_source(pickle_path, metadata_path)
    poses = load_poses(dataset_root / "poses.txt")
    projection = load_projection(dataset_root / "calib.txt")
    identity_document = {
        "pickle_sha256": file_sha256(pickle_path),
        "metadata_sha256": file_sha256(metadata_path),
        "poses_sha256": file_sha256(dataset_root / "poses.txt"),
        "calib_sha256": file_sha256(dataset_root / "calib.txt"),
        "model": arguments.model,
        "stride": arguments.stride,
        "start": arguments.start,
        "maximum_views": arguments.maximum_views,
        "minimum_landmark_confidence": arguments.minimum_landmark_confidence,
    }
    identity = hashlib.sha256(
        json.dumps(identity_document, sort_keys=True).encode("utf-8")
    ).hexdigest()
    descriptions = load_checkpoint(checkpoint_path, identity)
    view_records_path = work_directory / "instance_views.json"
    if view_records_path.is_file():
        view_document = strict_json_loads(
            view_records_path.read_text(encoding="utf-8"), "instance views"
        )
        if view_document.get("identity") != identity:
            raise InstanceDescriptionError("instance views belong to another run")
        view_records = view_document.get("views", {})
    else:
        view_records = {}
        for object_id, item in enumerate(objects):
            views = create_object_views(
                object_id,
                item,
                dataset_root,
                caption_directory,
                poses,
                projection,
                crop_root,
                arguments.stride,
                arguments.start,
                arguments.maximum_candidate_frames,
                arguments.maximum_views,
            )
            view_records[str(object_id)] = views
            if (object_id + 1) % 25 == 0 or object_id + 1 == len(objects):
                print("Prepared instance views {}/{}.".format(object_id + 1, len(objects)), flush=True)
        atomic_write(
            view_records_path,
            {"schema_version": 1, "identity": identity, "views": view_records},
        )

    client = OllamaSemanticClient(
        base_url=arguments.base_url,
        cache_path=arguments.cache,
        timeout=arguments.timeout,
    )
    pending = [
        {
            "object_id": object_id,
            "views": view_records.get(str(object_id), []),
        }
        for object_id in range(len(objects))
        if view_records.get(str(object_id)) and str(object_id) not in descriptions
    ]
    for offset in range(0, len(pending), arguments.batch_size):
        batch = pending[offset:offset + arguments.batch_size]
        validated = describe_records(
            client,
            arguments.model,
            batch,
            arguments.minimum_landmark_confidence,
            arguments.validation_retries,
        )
        for item in validated:
            descriptions[str(item["object_id"])] = item
        save_checkpoint(checkpoint_path, identity, descriptions)
        print(
            "Described {}/{} viewable instances ({} total OpenGraph objects).".format(
                min(offset + len(batch), len(pending)), len(pending), len(objects)
            ),
            flush=True,
        )

    promoted = [
        apply_local_landmark_policy(item)
        for item in descriptions.values()
        if apply_local_landmark_policy(item).get("landmark_usable") is True
    ]
    embedding_vectors = client.embed_texts(
        ["{}；类别：{}".format(item["caption_zh"], item["category_zh"]) for item in promoted],
        model=arguments.embedding_model,
        dimensions=arguments.embedding_dimensions,
    ) if promoted else []
    embeddings = {
        str(item["object_id"]): vector
        for item, vector in zip(promoted, embedding_vectors)
    }
    output = build_output(
        metadata,
        descriptions,
        view_records,
        pickle_path,
        metadata_path,
        dataset_root,
        arguments.model,
        arguments.embedding_model,
        arguments.embedding_dimensions,
        arguments.minimum_landmark_confidence,
        embeddings,
    )
    atomic_write(output_path, output)
    print(
        "Saved {} Chinese descriptions and {} promoted landmarks from {} objects to {}.".format(
            output["described_object_count"],
            output["landmark_count"],
            output["object_count"],
            output_path,
        ),
        flush=True,
    )


if __name__ == "__main__":
    try:
        main()
    except (
        OllamaSemanticError,
        InstanceDescriptionError,
        OSError,
        UnicodeDecodeError,
        ValueError,
        pickle.UnpicklingError,
    ) as error:
        raise SystemExit("error: {}".format(error)) from error
