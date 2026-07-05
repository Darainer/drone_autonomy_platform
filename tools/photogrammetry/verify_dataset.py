#!/usr/bin/env python3
"""verify_dataset.py — standalone DES-004 dataset validator (offload verification aid).

CLI: python verify_dataset.py <dataset_dir>

Exits 0 iff the dataset directory is a valid, complete DES-004 survey
dataset. Checks, in order:
  (a) a finalized manifest.yaml is present and parses with the required
      schema fields (a dataset with only manifest.yaml.part is invalid);
  (b) poses.csv is present, has the required columns, all rows parse, and
      frame_idx is contiguous starting at the first row's index;
  (c) every file listed in the manifest's sha256 map matches its on-disk
      sha256 (this is what catches a corrupted/truncated image byte, and
      names the offending file);
  (d) every image referenced by poses.csv exists on disk.

Any failure is printed with a specific reason (naming the offending file
where applicable) and the process exits non-zero. This script intentionally
duplicates none of the reader logic -- it is a thin CLI over
photogrammetry.dataset.
"""
from __future__ import annotations

import argparse
import pathlib
import sys
from typing import List

from PIL import Image

from photogrammetry import dataset as ds


def verify_dataset(dataset_dir) -> List[str]:
    """Run all DES-004 validation checks. Returns a list of error strings (empty = valid)."""
    dataset_dir = pathlib.Path(dataset_dir)
    errors: List[str] = []

    try:
        manifest = ds.load_manifest(dataset_dir)
    except ds.DatasetError as exc:
        errors.append(str(exc))
        # Without a valid manifest there is nothing else meaningful to check
        # (no sha256 map to verify files against).
        return errors

    poses = None
    try:
        poses = ds.read_poses(dataset_dir)
    except ds.DatasetError as exc:
        errors.append(str(exc))

    if poses is not None:
        try:
            ds.validate_frame_idx_contiguous(poses, start=poses[0].frame_idx)
        except ds.DatasetError as exc:
            errors.append(str(exc))

    # Checksum verification: every file recorded in the manifest must exist
    # on disk and match. This is what surfaces a corrupted image byte.
    sha256_map = manifest.get("sha256", {})
    for relpath, expected_hash in sha256_map.items():
        file_path = dataset_dir / relpath
        if not file_path.is_file():
            errors.append(f"file listed in manifest sha256 is missing on disk: {relpath}")
            continue
        actual_hash = ds.sha256_of_file(file_path)
        if actual_hash != expected_hash:
            errors.append(
                f"checksum mismatch for {relpath}: manifest={expected_hash} "
                f"actual={actual_hash} (file is corrupt or was modified after recording)"
            )

    # Every image referenced by poses.csv must exist on disk.
    if poses is not None:
        for row in poses:
            image_path = dataset_dir / ds.IMAGES_DIRNAME / row.image_filename
            if not image_path.is_file():
                errors.append(
                    f"image referenced by poses.csv frame_idx={row.frame_idx} is missing: "
                    f"{ds.IMAGES_DIRNAME}/{row.image_filename}"
                )

    return errors


def collect_warnings(dataset_dir) -> List[str]:
    """Non-fatal consistency warnings for an otherwise-loadable dataset (F-6(b)).

    Today the only check is: does the FIRST image referenced by poses.csv
    have on-disk pixel dimensions matching the manifest's declared
    camera_intrinsics width/height? A mismatch (e.g. an 8x8 placeholder JPEG
    against a manifest declaring 640x480) silently passes `verify_dataset()`
    today -- that check only validates schema/checksums/existence, not pixel
    content -- yet the same symptom would equally hide a recorder
    misconfiguration (wrong camera profile writing an intrinsics block that
    no longer matches what the camera is actually producing).

    The image is opened with Pillow HEADER-ONLY (`Image.open(...).size` reads
    the file header; pixel data is never decoded), so this stays cheap enough
    to call from check mode without violating the no-reconstruction
    invariant.

    This is deliberately forgiving of anything that would make it
    inconclusive rather than wrong: a missing/unreadable first image, or a
    manifest/poses that don't even load, just yields no warnings (`[]`) --
    surfacing THAT class of problem is verify_dataset()'s job (errors), not
    this function's (warnings).
    """
    dataset_dir = pathlib.Path(dataset_dir)

    try:
        manifest = ds.load_manifest(dataset_dir)
    except ds.DatasetError:
        return []

    try:
        poses = ds.read_poses(dataset_dir)
    except ds.DatasetError:
        return []

    if not poses:
        return []

    intrinsics = manifest.get("camera_intrinsics")
    if not isinstance(intrinsics, dict):
        return []
    try:
        manifest_width = int(intrinsics["width"])
        manifest_height = int(intrinsics["height"])
    except (KeyError, TypeError, ValueError):
        return []

    first_image_path = dataset_dir / ds.IMAGES_DIRNAME / poses[0].image_filename
    if not first_image_path.is_file():
        return []

    try:
        with Image.open(first_image_path) as img:
            actual_width, actual_height = img.size
    except OSError:
        return []

    if (actual_width, actual_height) != (manifest_width, manifest_height):
        return [
            f"image dimensions {actual_width}x{actual_height} of "
            f"{ds.IMAGES_DIRNAME}/{poses[0].image_filename} do not match manifest "
            f"camera_intrinsics {manifest_width}x{manifest_height} -- check for a "
            f"recorder camera-profile misconfiguration"
        ]
    return []


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Validate a DES-004 survey dataset directory.")
    parser.add_argument("dataset_dir", help="Path to a survey_<mission_id>/ dataset directory")
    args = parser.parse_args(argv)

    errors = verify_dataset(args.dataset_dir)
    if errors:
        print(f"INVALID dataset: {args.dataset_dir}")
        for err in errors:
            print(f"  - {err}")

    # Non-fatal consistency warnings (F-6(b)) -- printed regardless of the
    # error outcome above; they never change the exit code.
    for warning in collect_warnings(args.dataset_dir):
        print(f"WARNING: {warning}", file=sys.stderr)

    if errors:
        return 1

    print(f"OK: {args.dataset_dir} is a valid DES-004 dataset")
    return 0


if __name__ == "__main__":
    sys.exit(main())
