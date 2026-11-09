"""Mount the HIL RTK antennas on the deck instead of tall CAD posts."""
import hashlib
import json
from pathlib import Path
import shutil


VERTICAL_POSTS = frozenset(('Cube1', 'Cube2'))
# The original vertical posts span exactly 520 mm, between the deck rail and
# the short antenna feet. Remove that rise from both measurement frames.
RTK_LOWERING_M = .52
RTK_SUPPORTS = frozenset(('RTK', 'Cube1', 'Cube2', 'Plane1', 'Plane2',
                          'HandleCollar', 'HandleFoot', 'HandleStem'))
RTK_SHORT_STEMS = frozenset(('HandleCollar', 'HandleFoot', 'HandleStem'))


def antenna_heads_only(obj, baseline_m=.52, retain_short_stems=False):
    """Extract housings with optional original short stems, centred at z=0."""
    omitted = RTK_SUPPORTS-RTK_SHORT_STEMS if retain_short_stems else RTK_SUPPORTS
    current, head = None, -1
    removed, output = {}, []
    for line in obj.splitlines(keepends=True):
        fields = line.split()
        if fields and fields[0] == 'o':
            current = ' '.join(fields[1:])
            if current == 'Base':
                head += 1
        if current in omitted and fields and fields[0] in ('f', 'l', 'p'):
            removed[current] = removed.get(current, 0)+1
            continue
        if fields and fields[0] == 'v' and current not in omitted:
            if head not in (0, 1):
                raise ValueError('Expected antenna Base object before housing geometry')
            # The export lists the secondary housing first (centred at y=-.65).
            fields[2] = f'{float(fields[2])+(1-head)*(.65-baseline_m):.12g}'
            fields[3] = f'{float(fields[3])-.10:.12g}'
            line = ' '.join(fields)+'\n'
        # Keep every index table, even for omitted supports; later faces use
        # both absolute and negative OBJ indices.
        output.append(line)
    if head != 1 or set(removed) != omitted:
        raise ValueError(f'Expected two antenna heads and all support objects: {removed}')
    return ''.join(output), removed


def prepare_antenna_heads(generated, directory, baseline_m=.52, retain_short_stems=False):
    source = Path(generated)/'models/sensor_rtk_master_visual.obj'
    destination = Path(directory)/'assets/rtk'
    destination.mkdir(parents=True, exist_ok=True)
    original = source.read_bytes()
    filtered, removed = antenna_heads_only(original.decode('utf-8'), baseline_m, retain_short_stems)
    mesh = destination/('antennas_short_stems.obj' if retain_short_stems else 'antenna_heads.obj')
    mesh.write_text(filtered)
    shutil.copy2(source.with_suffix('.mtl'), destination/source.with_suffix('.mtl').name)
    report = dict(source=str(source.resolve()), source_sha256=hashlib.sha256(original).hexdigest(),
                  result_sha256=hashlib.sha256(mesh.read_bytes()).hexdigest(),
                  removed_primitives=removed, baseline_m=baseline_m,
                  short_stems_retained=retain_short_stems,
                  scope=('Two housings with original 62 mm stems/feet; no tall posts or rails'
                         if retain_short_stems else 'Two housings only; all posts/rails/feet removed'))
    (destination/'provenance.json').write_text(json.dumps(report, indent=2)+'\n')
    return mesh.resolve()


def without_vertical_posts(obj, lift_crossbar_m=0.):
    current = None
    removed = dict.fromkeys(VERTICAL_POSTS, 0)
    output = []
    for line in obj.splitlines(keepends=True):
        fields = line.strip().split(maxsplit=1)
        if fields and fields[0] == 'o':
            current = fields[1] if len(fields) == 2 else None
            if current in VERTICAL_POSTS:
                output.append(f'# HIL omitted vertical post: {current}\n')
                continue
        if current in VERTICAL_POSTS and fields and fields[0] in ('f', 'l', 'p'):
            removed[current] += 1
            continue
        if current == 'RTK' and fields and fields[0] == 'v' and lift_crossbar_m:
            coordinates = line.split()
            coordinates[3] = f'{float(coordinates[3])+lift_crossbar_m:.12g}'
            line = ' '.join(coordinates)+'\n'
        # Preserve all index tables and material state, including negative OBJ
        # indices used by subsequent objects. Only the named posts lose faces.
        output.append(line)
    if not all(removed.values()):
        raise ValueError(f'Expected two RTK vertical post objects with geometry: {removed}')
    return ''.join(output), removed


def prepare_rtk_visual(generated, directory):
    source = Path(generated) / 'models/sensor_rtk_master_visual.obj'
    destination = Path(directory) / 'assets/rtk'
    destination.mkdir(parents=True, exist_ok=True)
    original = source.read_bytes()
    # The complete RTK frame moves down. Compensate only the retained bottom
    # rail so it stays on the chassis rather than sinking below the vehicle.
    filtered, removed = without_vertical_posts(original.decode('utf-8'), RTK_LOWERING_M)
    mesh = destination / 'sensor_rtk_deck_mount.obj'
    mesh.write_text(filtered)
    shutil.copy2(source.with_suffix('.mtl'), destination / source.with_suffix('.mtl').name)
    report = {'source': str(source.resolve()),
              'source_sha256': hashlib.sha256(original).hexdigest(),
              'result_sha256': hashlib.sha256(mesh.read_bytes()).hexdigest(),
              'removed_primitives': removed,
              'antenna_lowering_m': RTK_LOWERING_M,
              'scope': 'Cube1/Cube2 removed; both antenna frames lowered 520 mm onto the deck; retained bottom rail stays at its original height'}
    (destination / 'provenance.json').write_text(json.dumps(report, indent=2)+'\n')
    return mesh.resolve()
