import json
from pathlib import Path

import numpy as np
import pytest

from agribot_mobile_app.catalog import CatalogError, GridData
from agribot_mobile_app.saved_path import (
    SavedPathStore, blocked_ahead, forward_index, numbered_markers, rejoin_indices, sha256,
)


def straight():
    return [{"x": i/10, "y": 0., "yaw": 0.} for i in range(101)]


def fixture_store(tmp_path):
    base = tmp_path/"map"
    files = {"pgm":base.with_suffix(".pgm"), "yaml":base.with_suffix(".yaml"),
             "georeference":tmp_path/"map_georeference.yaml", "nav2":tmp_path/"nav2.yaml",
             "primitives":tmp_path/"motion_primitives/diff_5cm.json"}
    for name,p in files.items():
        p.parent.mkdir(parents=True,exist_ok=True)
        p.write_text(name)
    class Catalog:
        def map_base(self, identifier):
            return base
    store = SavedPathStore(Catalog(), files["nav2"])
    doc = {"schema_version":1,"map_id":"map","fingerprints":{k:sha256(p) for k,p in files.items()},
           "routes":[{"id":"test","label":"Test","vehicle_type":"differential",
                      "path":straight(),"anchor_path_indices":[0,30,100]}]}
    output = tmp_path/"map_saved_paths.json"
    output.write_text(json.dumps(doc))
    return store,files,doc,output


def test_select_suffix_preserves_dense_path_and_hides_prefix(tmp_path):
    store,files,doc,output=fixture_store(tmp_path)
    route,path=store.select("map","test",2)
    assert path==straight()[30:]
    assert len(store.public("map")["routes"])==1
    assert "path" not in store.public("map")["routes"][0]
    for bad in (0,3,4,True,1.2):
        with pytest.raises(CatalogError):store.select("map","test",bad)


@pytest.mark.parametrize("kind",["pgm","yaml","georeference","nav2","primitives"])
def test_changed_map_or_config_rejects_saved_path(tmp_path,kind):
    store,files,doc,output=fixture_store(tmp_path)
    files[kind].write_text("changed")
    with pytest.raises(CatalogError):store.load("map")


def test_progress_never_jumps_to_return_leg():
    path=straight()+[{"x": i/10,"y":.05,"yaw":3.14} for i in range(100,-1,-1)]
    assert forward_index(path,{"x":1,"y":.05},10)<30
    assert forward_index(path,{"x":0,"y":0},30)>=30
    assert all(i>10 for i in rejoin_indices(path,10))


def test_preview_numbers_keep_original_sequence_and_do_not_number_dense_samples():
    route={"path":[{"x":i*.05,"y":0.,"yaw":0.} for i in range(1040)],
           "anchor_path_indices":[i*10 for i in range(104)]}
    markers=numbered_markers(route,51)
    assert [p["number"] for p in markers]==list(range(51,105))
    assert markers[0]["x"]==25.0
    assert markers[-1]["x"]==51.5
    assert len(numbered_markers(route,1))==104


def test_filled_footprint_blockage_and_out_of_bounds_but_not_unknown():
    pixels=np.zeros((100,160),dtype=np.int8)
    pixels[49:51,50:52]=100
    grid=GridData(160,100,.1,-3,-5,0,pixels.tobytes())
    footprint=[[.7,.53],[.7,-.53],[-.7,-.53],[-.7,.53]]
    assert blocked_ahead(straight(),0,grid,footprint)
    assert not blocked_ahead(straight(),60,grid,footprint,distance=0)
    outside=[{"x":20., "y":0., "yaw":0.}]
    assert blocked_ahead(outside,0,grid,footprint)
    # A distant rejoin endpoint is checked by Smac, not rejected merely because
    # it is outside the small rolling window.
    assert not blocked_ahead(outside,0,grid,footprint,outside_is_blocked=False)
    unknown=GridData(160,100,.1,-3,-5,0,bytes([255])*16000)
    assert not blocked_ahead(straight(),0,unknown,footprint)
    assert not blocked_ahead(straight(),0,unknown,footprint,outside_is_blocked=False)


def test_one_meter_monitor_ignores_farther_obstacle_but_detects_near_obstacle():
    pixels=np.zeros((100,160),dtype=np.int8)
    # x=2.0m is outside the 1m centerline horizon plus the front footprint.
    pixels[49:51,50:52]=100
    grid=GridData(160,100,.1,-3,-5,0,pixels.tobytes())
    footprint=[[.7,.53],[.7,-.53],[-.7,-.53],[-.7,.53]]
    assert not blocked_ahead(straight(),0,grid,footprint,distance=1.0)
    assert blocked_ahead(straight(),0,grid,footprint,distance=3.0)
    pixels[49:51,43:45]=100
    near=GridData(160,100,.1,-3,-5,0,pixels.tobytes())
    assert blocked_ahead(straight(),0,near,footprint,distance=1.0)


def test_known_obstacle_inside_unknown_area_still_blocks():
    pixels=np.full((100,160),-1,dtype=np.int8)
    pixels[49:51,43:45]=100
    grid=GridData(160,100,.1,-3,-5,0,pixels.tobytes())
    footprint=[[.7,.53],[.7,-.53],[-.7,-.53],[-.7,.53]]
    assert blocked_ahead(straight(),0,grid,footprint,distance=1.0)
