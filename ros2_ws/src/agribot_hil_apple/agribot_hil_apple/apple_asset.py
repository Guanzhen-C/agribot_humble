"""Bake the scan's measured stem socket to the origin, with stem axis +Z."""
import json
import shutil
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

NAMESPACE = 'http://www.collada.org/2005/11/COLLADASchema'


def bake_stem_up(source, directory):
    directory.mkdir(parents=True, exist_ok=True)
    tree = ET.parse(source)
    ns = {'c': NAMESPACE}
    mesh = tree.find('.//c:mesh', ns)
    positions = mesh.find("c:source[@id='shape0-lib-positions']/c:float_array", ns)
    normals = mesh.find("c:source[@id='shape0-lib-normals']/c:float_array", ns)
    points = np.fromstring(positions.text, sep=' ').reshape(-1, 3)
    vectors = np.fromstring(normals.text, sep=' ').reshape(-1, 3)
    # Measured from vertices associated with the scan's green stem UV region.
    socket = np.array([-.00018581, -.00598261, .06426795])
    center = .5 * (points.min(axis=0) + points.max(axis=0))
    axis = socket - center
    axis /= np.linalg.norm(axis)
    tangent = np.array([1., 0., 0.]) - axis * axis[0]
    tangent /= np.linalg.norm(tangent)
    rotation, _ = Rotation.align_vectors([[0., 0., 1.], [1., 0., 0.]], [axis, tangent])
    baked = rotation.apply(points - socket)
    positions.text = ' '.join(f'{v:.9g}' for v in baked.ravel())
    normals.text = ' '.join(f'{v:.9g}' for v in rotation.apply(vectors).ravel())
    tree.find('c:asset/c:up_axis', ns).text = 'Z_UP'
    ET.register_namespace('', NAMESPACE)
    destination = directory / 'textured_stem_up.dae'
    tree.write(destination, encoding='utf-8', xml_declaration=True)
    shutil.copy2(source.parent / 'texture_map.png', directory / 'texture_map.png')
    report = {'source_stem_socket': socket.tolist(), 'source_stem_axis': axis.tolist(),
              'baked_socket': rotation.apply(socket-socket).tolist(),
              'baked_stem_axis': rotation.apply(axis).tolist(),
              'bounds': [baked.min(axis=0).tolist(), baked.max(axis=0).tolist()]}
    (directory / 'alignment.json').write_text(json.dumps(report, indent=2) + '\n')
    return destination
