#!/usr/bin/env python3
"""静态回归：防止显示配置重新订阅非法输入或以透明自由格叠加黑墙。"""
from pathlib import Path
import yaml


def main():
    path = Path(__file__).resolve().parents[1] / 'rviz' / 'fusion_test.rviz'
    with path.open(encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    displays = config['Visualization Manager']['Displays']
    topics = {item.get('Topic', {}).get('Value'): item for item in displays}
    prefix = '/passage_fusion_test/'
    assert prefix + 'test_evidence' not in topics, 'Raw malformed evidence must never reach RViz'
    original = topics[prefix + 'map']
    final = topics[prefix + 'costmap']
    assert not original['Enabled'] and not original['Value'], 'Original wall must be hidden by default'
    assert final['Enabled'] and final['Value']
    assert final['Color Scheme'] == 'map' and final['Alpha'] == 1
    assert prefix + 'evidence_preview' in topics
    assert topics[prefix + 'status_marker']['Enabled']
    print('PASS: RViz isolation, opaque final map and raw-cost labels')


if __name__ == '__main__':
    main()
