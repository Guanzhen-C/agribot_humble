import pytest

from agribot_farm_sim.mesh_surface import Surface, load_surfaces


def test_surface_interpolates_instead_of_using_nearby_vertex_maximum():
    surface = Surface([[(0,0,0), (1,0,1), (0,1,0)]])
    assert surface.height(.2, .2) == pytest.approx(.2)
    with pytest.raises(ValueError, match="directly below"):
        surface.height(.9, .9)


def test_highest_surface_and_vertical_faces():
    surface = Surface([[(0,0,1), (1,0,1), (0,1,1)],
                       [(0,0,2), (1,0,2), (0,1,2)],
                       [(0,0,0), (0,0,3), (0,1,3)]])
    assert surface.height(0, .2) == 2


def test_obj_part_selection_quad_and_negative_indices(tmp_path):
    mesh = tmp_path / "sample.obj"
    mesh.write_text("v 0 0 1\nv 1 0 1\nv 1 1 1\nv 0 1 1\n"
                    "o hood\nf -4/1/1 -3/1/1 -2/1/1 -1/1/1\n"
                    "o unused\nf 1 2 3\n")
    surfaces = load_surfaces(mesh, {"hood", "missing"})
    assert surfaces["hood"].height(.8, .8) == 1
    with pytest.raises(ValueError, match="no triangles"):
        surfaces["missing"].height(0, 0)
