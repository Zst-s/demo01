#!/usr/bin/env python3
"""Persistent geometry, migration and observed-free static-region regression checks."""
from dataclasses import replace
from pathlib import Path
import json, tempfile, unittest
from unittest.mock import patch
from global_passage_memory import MapContext
from passage_memory_v2 import PassageMemoryV2
from continuous_passage_memory import ContinuousPassageMemory
from passage_record_handover import export_record
from passage_record_io import read_record


class ContinuousMemoryTests(unittest.TestCase):
    def setUp(self):
        self.context=MapContext('mapping-test','map',10,10,.05,0,0,0,0,0)
        self.grid=[100]*100
        self.observation=dict(frame='odom',measured_at=10.,resolution=.05,
            columns=[(.025+x*.05,.025+y*.05,-.2,.2) for y in range(4) for x in range(4)])
    def legacy(self,path):
        old=PassageMemoryV2();old.add_structure(context=self.context,grid=self.grid,
            observation=self.observation,cells=tuple(y*10+x for y in range(4) for x in range(4)),
            map_transform=(0,0,0,0),now=10.1);old.save(path)
    def test_v3_migration_keeps_original_and_roundtrips_v4(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'runtime.json';self.legacy(p);original=p.read_bytes()
            m=ContinuousPassageMemory.restore(p,self.context,self.grid)
            self.assertEqual(m.state,'KNOWN_PASSAGE');self.assertTrue(m.needs_save)
            groups={k:set(v) for k,v in m.groups.items()};m.save(p)
            self.assertEqual(Path(m.backup_path).read_bytes(),original)
            self.assertEqual(read_record(p)['version'],4)
            restored=ContinuousPassageMemory.restore(p,self.context,self.grid)
            self.assertEqual(restored.groups,groups);self.assertEqual(restored.mask_cells,m.mask_cells)
            self.assertFalse(restored.segments)
    def test_mapping_changes_keep_coverage_without_new_structure(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'runtime.json';self.legacy(p);m=ContinuousPassageMemory.restore(p,self.context,self.grid)
            expected=m.planning_cells(self.context,self.grid)
            for value in (0,-1,100):
                grid=self.grid.copy();grid[0]=value
                self.assertEqual(m.planning_cells(self.context,grid),expected if value==100 else expected-{0})
                m.save(p)
            self.assertEqual(m.planning_cells(self.context,self.grid),expected)
    def test_expansion_epoch_and_tf_guards(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'runtime.json';self.legacy(p);m=ContinuousPassageMemory.restore(p,self.context,self.grid)
            grown=replace(self.context,width=12,height=12,origin_x=-.05,origin_y=-.05)
            self.assertTrue(m.check(grown,[100]*144));self.assertEqual(len(m.mask_cells),16)
            self.assertFalse(m.check(replace(grown,epoch='mapping-new'),[100]*144))
            m=ContinuousPassageMemory.restore(p,self.context,self.grid)
            self.assertFalse(m.check(replace(self.context,map_to_odom_x=1),self.grid))
    def test_growing_canvas_small_edge_retraction_preserves_world_history(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'runtime.json';self.legacy(p)
            for shift in (.005818,.007059,.021622,.05):
                m=ContinuousPassageMemory.restore(p,self.context,self.grid)
                groups={k:set(v) for k,v in m.groups.items()};regions=set(m.regions)
                grown=replace(self.context,width=12,height=12,origin_x=shift)
                self.assertTrue(m.check(grown,[100]*144))
                self.assertEqual(m.groups,groups);self.assertEqual(m.regions,regions)
                m.save(Path(d)/'grown.json')
                restored=ContinuousPassageMemory.restore(Path(d)/'grown.json',grown,[100]*144)
                self.assertEqual(restored.groups,groups);self.assertEqual(restored.regions,regions)
            for bad in (replace(self.context,width=12,height=12,origin_x=.051),
                        replace(self.context,origin_x=.01),
                        replace(self.context,width=9,height=12),
                        replace(self.context,width=12,height=12,epoch='mapping-other'),
                        replace(self.context,width=12,height=12,map_to_odom_x=.3)):
                m=ContinuousPassageMemory.restore(p,self.context,self.grid)
                self.assertFalse(m.check(bad,[100]*(bad.width*bad.height)))

    def test_duplicates_do_not_allocate_new_groups_and_new_height_is_retained(self):
        m=ContinuousPassageMemory();args=dict(context=self.context,grid=self.grid,cells=(0,),
            observation=self.observation,map_transform=(0,0,0,0),now=10.1)
        self.assertIsNotNone(m.add_structure(**args));before=sum(len(v) for v in m.groups.values())
        self.assertIsNone(m.add_structure(**args));self.assertEqual(sum(len(v) for v in m.groups.values()),before)
        obs=dict(self.observation,columns=[(.025,.025,-.2,.25)])
        self.assertIsNotNone(m.add_structure(**dict(args,observation=obs)))
        self.assertEqual(sum(len(v) for v in m.groups.values()),before+1)
    def test_write_failure_suspends_and_preserves_old_file(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'runtime.json';self.legacy(p);original=p.read_bytes();m=ContinuousPassageMemory.restore(p,self.context,self.grid)
            with patch('passage_record_io.os.replace',side_effect=OSError('disk unavailable')):
                with self.assertRaises(OSError):m.save(p)
            self.assertEqual(p.read_bytes(),original);self.assertEqual(m.state,'SUSPENDED')
            self.assertFalse(m.planning_cells(self.context,self.grid))
    def test_v4_handover_rebases_tf_and_preserves_saved_map_guard(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'runtime.json';self.legacy(p);m=ContinuousPassageMemory.restore(p,self.context,self.grid);m.save(p)
            saved='saved-map-'+'a'*64;out=Path(d)/'saved.json';self.assertEqual(export_record(p,out,saved),1)
            ctx=replace(self.context,epoch=saved,map_to_odom_x=5)
            restored=ContinuousPassageMemory.restore(out,ctx,self.grid)
            self.assertEqual(restored.state,'KNOWN_PASSAGE');self.assertEqual(len(restored.mask_cells),16)
            changed=self.grid.copy();changed[0]=0
            self.assertFalse(restored.check(ctx,changed))
    def test_new_mapping_session_archives_previous_v4_before_replacing(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'runtime.json';self.legacy(p);m=ContinuousPassageMemory.restore(p,self.context,self.grid);m.save(p)
            previous=p.read_bytes();fresh=ContinuousPassageMemory();new_context=replace(self.context,epoch='mapping-next')
            fresh.add_structure(context=new_context,grid=self.grid,observation=self.observation,cells=(0,),map_transform=(0,0,0,0),now=10.1)
            fresh.save(p)
            self.assertEqual(Path(fresh.backup_path).read_bytes(),previous)
            self.assertEqual(read_record(p)['context']['epoch'],'mapping-next')

    def test_old_body_proof_is_not_promoted_to_hollow(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'runtime.json';m=PassageMemoryV2();m.add(context=self.context,grid=self.grid,
                center_x=.125,center_y=.125,yaw=0,travel_length=.1,robot_length=.1,robot_width=.1,
                confirmed_frames=3,measured_at=10.,now=10.1);m.save(p)
            original=p.read_bytes();new=ContinuousPassageMemory.restore(p,self.context,self.grid)
            self.assertFalse(new.groups);self.assertFalse(new.segments);self.assertEqual(p.read_bytes(),original)

if __name__=='__main__':unittest.main(verbosity=2)
