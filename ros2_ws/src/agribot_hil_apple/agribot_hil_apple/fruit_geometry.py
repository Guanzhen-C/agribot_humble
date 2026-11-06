"""Shared physical stem dimensions for scene generation and collision planning."""


def stem_dimensions(socket_to_branch=.04):
    if not .03 <= socket_to_branch <= .08:
        raise ValueError('Unsupported fruit stem length')
    return {'visual_center': socket_to_branch/2.,
            'visual_length': socket_to_branch+.006,
            'collision_center': (socket_to_branch-.005)/2.,
            'collision_length': socket_to_branch-.007}
