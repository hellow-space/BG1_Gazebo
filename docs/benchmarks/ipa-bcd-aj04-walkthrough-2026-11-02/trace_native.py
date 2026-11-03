#!/usr/bin/env python3
"""Instrument disposable IPA copies; assert the saved native result is unchanged."""
import hashlib
import json
from pathlib import Path
import shlex
import subprocess
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path.insert(0, str(ROOT))
from coverage_benchmark.hybrid.build import build
from coverage_benchmark.provenance import file_sha256

SCENE = 'A11123AJ04_231127_094529'
HISTORY = ROOT / 'docs/benchmarks/hybrid-local-bcd-2026-10-30'

HEADER = r'''
#pragma once
#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>
#include <fstream>
extern nlohmann::json bcd_trace;
extern std::string bcd_trace_dir;
template<class T> inline nlohmann::json trace_points(const std::vector<T>& pts) {
 nlohmann::json j=nlohmann::json::array();
 for(const auto& p:pts)j.push_back({p.x,p.y});return j;
}
inline nlohmann::json trace_matrix(const cv::Mat& mat) {
 nlohmann::json j=nlohmann::json::array();
 for(int y=0;y<mat.rows;++y){nlohmann::json row=nlohmann::json::array();
 for(int x=0;x<mat.cols;++x)row.push_back(mat.at<double>(y,x));j.push_back(row);}return j;
}
inline void trace_image(const std::string& name,const cv::Mat& image) {
 if(!cv::imwrite(bcd_trace_dir+"/"+name+".png",image))throw std::runtime_error("trace image failed");
}
inline void trace_labels(const std::string& name,const cv::Mat& image) {
 cv::Mat positive,out;cv::max(image,0,positive);positive.convertTo(out,CV_16U);trace_image(name,out);
}
'''


def once(text, old, new):
    if text.count(old) != 1:
        raise ValueError(f'Expected one source anchor ({text.count(old)}): {old[:100]}')
    return text.replace(old, new, 1)


def instrument_bcd(text):
    # The upstream file ends with an entirely commented alternative decomposition.
    # Keep it verbatim, but do not treat its commented statements as live anchors.
    text, disabled = text.split('//void BoustrophedonVariantExplorer::computeCellDecomposition(', 1)
    text = once(text, 'cv::Mat cell_map = room_map.clone();',
                'cv::Mat cell_map = room_map.clone();\n trace_image("01_rotated_room",room_map);')
    text = once(text, '// sweep trough the map and detect critical points',
                'bcd_trace["first_scan_row"] = y_start;\n bcd_trace["initial_segments"]=previous_number_of_segments;\n'
                'bcd_trace["rows"].push_back({{"y",y_start},{"segments",previous_number_of_segments}});\n'
                '// sweep trough the map and detect critical points')
    text = once(text, '// reset hit_white_pixel to use this Boolean later', r'''
        const bool trace_event = previous_number_of_segments != number_of_segments || segment_shift_detected;
        cv::Mat trace_before;
        if(trace_event)cell_map.copyTo(trace_before);
        nlohmann::json trace_critical=nlohmann::json::array();
        bcd_trace["rows"].push_back({{"y",y},{"segments",number_of_segments}});
        // reset hit_white_pixel to use this Boolean later''')
    # The first anchor belongs to an IN/SHIFT event, the second to OUT.
    anchor = 'if(critical_point == true)\n\t\t\t\t\t{'
    assert text.count(anchor) == 2
    text = text.replace(anchor, anchor + '\n trace_critical.push_back({x,(int)y});', 1)
    start = text.index('else if(previous_number_of_segments > number_of_segments)')
    text = text[:start] + text[start:].replace(anchor, anchor + '\n trace_critical.push_back({x,(int)y-1});', 1)
    text = once(text, '// save the found number of segments and the obstacle end points', r'''
        if(trace_event) {
          nlohmann::json cuts=nlohmann::json::array();
          const int cy=(previous_number_of_segments > number_of_segments ? (int)y-1 : (int)y);
          int begin=-1;
          for(int x=0;x<=cell_map.cols;++x) {
            bool added=x<cell_map.cols && cell_map.at<uchar>(cy,x)==BORDER_PIXEL_VALUE && trace_before.at<uchar>(cy,x)!=BORDER_PIXEL_VALUE;
            if(added && begin<0)begin=x;
            if(!added && begin>=0){cuts.push_back({begin,cy,x-1,cy});begin=-1;}
          }
          bcd_trace["events"].push_back({{"y",y},{"before",previous_number_of_segments},
            {"after",number_of_segments},{"type",segment_shift_detected?"SHIFT":(previous_number_of_segments<number_of_segments?"IN":"OUT")},
            {"critical_points",trace_critical},{"cuts",cuts}});
        }
        // save the found number of segments and the obstacle end points''')
    text = once(text, 'const int number_of_cells = mergeCells(cell_map, cell_map_labels, min_cell_area, min_cell_width);',
                'trace_image("02_cuts",cell_map);\n'
                'const int number_of_cells = mergeCells(cell_map, cell_map_labels, min_cell_area, min_cell_width);\n'
                'trace_labels("04_labels_final",cell_map_labels);\n bcd_trace["final_label_count"]=number_of_cells;')
    text = once(text, '// determine the neighborhood relationships between all cells',
                'trace_labels("03_labels_before_merge",cell_map_labels);\n bcd_trace["before_merge_count"]=label_index-1;\n'
                '// determine the neighborhood relationships between all cells')
    text = once(text, '// iteratively merge cells\n\tmergeCellsSelection',
                'bcd_trace["adjacency"]=nlohmann::json::array();\n'
                'for(const auto& item:cell_index_mapping){nlohmann::json neighbors=nlohmann::json::array();\n'
                'for(const auto& n:item.second->neighbors_)neighbors.push_back(n->label_);\n'
                'bcd_trace["adjacency"].push_back({{"label",item.first},{"neighbors",neighbors},{"pixels",item.second->area_}});}\n'
                '// iteratively merge cells\n\tmergeCellsSelection')
    text = once(text, '// get a map that has only the current cell drawn in', r'''
    const int trace_id=bcd_trace["sweeps"].size();
    bcd_trace["sweeps"].push_back({{"id",trace_id},{"polygon_rotated",trace_points(cell.getVertices())},
      {"center_rotated",{cell.getCenter().x,cell.getCenter().y}},{"grid_spacing",grid_spacing_as_int},
      {"half_grid_spacing",half_grid_spacing_as_int},{"path_eps",path_eps},
      {"max_deviation",max_deviation_from_track},{"obstacle_offset",grid_obstacle_offset}});
    auto& trace_cell=bcd_trace["sweeps"][trace_id];
    // get a map that has only the current cell drawn in''')
    text = once(text, '// create inflated obstacles room map and rotate according to cell',
                'trace_cell["R_cell"]=trace_matrix(R_cell);\n'
                'trace_cell["rotated_shape"]={rotated_cell_map.cols,rotated_cell_map.rows};\n'
                'trace_cell["rotated_white_pixels"]=cv::countNonZero(rotated_cell_map);\n'
                '// create inflated obstacles room map and rotate according to cell')
    text = once(text, '// if no edge could be found in the cell (e.g. if it is too small), ignore it', r'''
    trace_cell["grid"]=nlohmann::json::array();
    for(const auto& line:grid_lines)trace_cell["grid"].push_back({{"upper",trace_points(line.upper_line)},
       {"lower",trace_points(line.lower_line)},{"two",line.has_two_valid_lines}});
    // if no edge could be found in the cell (e.g. if it is too small), ignore it''')
    text = once(text, 'int min_corner_index = 0;',
                'int min_corner_index = 0;\n trace_cell["corner_costs"]={min_corner_dist};')
    text = once(text, 'if (dist < min_corner_dist)',
                'trace_cell["corner_costs"].push_back(dist);\n if (dist < min_corner_dist)')
    text = once(text, 'bool start_from_upper_path = (min_corner_index<2 ? true : false);',
                'trace_cell["corners_rotated"]=trace_points(outer_corners);\n trace_cell["chosen_corner"]=min_corner_index;\n'
                'bool start_from_upper_path = (min_corner_index<2 ? true : false);')
    text = once(text, '// remap the fov path to the originally rotated cell and add the found points to the global path',
                'trace_cell["ordered_local_path"]=trace_points(current_fov_path);\n'
                '// remap the fov path to the originally rotated cell and add the found points to the global path')
    return text + '//void BoustrophedonVariantExplorer::computeCellDecomposition(' + disabled


def instrument_rotation(text):
    text = once(text, 'return rotation_angle;', r'''
    bcd_trace["rotations"].push_back({{"angle_rad",rotation_angle},{"R",trace_matrix(R)},
       {"center",{center_of_rotation.x,center_of_rotation.y}},
       {"shape",{bounding_rect.width,bounding_rect.height}}});
    return rotation_angle;''')
    text = once(text, 'return direction_histogram.getMaxBinPreciseVal();', r'''
    bcd_trace["directions"].push_back({{"lines",nlohmann::json::array()},{"min_line_length",min_line_length},
       {"angle_rad",direction_histogram.getMaxBinPreciseVal()}});
    for(const auto& line:lines)bcd_trace["directions"].back()["lines"].push_back({line[0],line[1],line[2],line[3]});
    return direction_histogram.getMaxBinPreciseVal();''')
    return text


def main():
    record = json.loads((HISTORY / f'{SCENE}_ipa_bcd.json').read_text())
    ev = record['details']['decomposition']['evidence']
    for path, digest in ev['files_sha256'].items():
        assert file_sha256(Path(path)) == digest, f'Historical native evidence changed: {path}'
    original = next(Path(p) for p in ev['files_sha256'] if p.endswith('/output.json'))
    _, info = build(ROOT)
    command = shlex.split(info['compile_command'])
    key = hashlib.sha256((Path(__file__).read_text() + json.dumps(info, sort_keys=True)).encode()).hexdigest()[:16]
    work = ROOT / 'coverage_benchmark/.build/ipa_bcd_report' / key
    work.mkdir(parents=True, exist_ok=True)
    out = HERE / 'evidence/native_trace'
    out.mkdir(parents=True, exist_ok=True)
    header = work / 'trace.hpp'
    header.write_text(HEADER)
    main_path = ROOT / 'coverage_benchmark/hybrid/native/main.cpp'
    main_text = main_path.read_text()
    main_text = main_text.replace('using json=nlohmann::json;',
        'using json=nlohmann::json;\njson bcd_trace={{"events",json::array()},{"sweeps",json::array()},'
        '{"rotations",json::array()},{"directions",json::array()},{"rows",json::array()}};\nstd::string bcd_trace_dir;')
    main_text = once(main_text, 'std::ifstream input(argv[1]);json req;input>>req;',
                     'bcd_trace_dir=std::string(argv[2]).substr(0,std::string(argv[2]).find_last_of("/"));\n'
                     'std::ifstream input(argv[1]);json req;input>>req;')
    main_text = once(main_text, 'std::ofstream output(argv[2]);',
                     'std::ofstream trace_output(bcd_trace_dir+"/trace.json");trace_output<<bcd_trace.dump();\n'
                     'std::ofstream output(argv[2]);')
    generated = {}
    for arg in command:
        p = Path(arg)
        if p.suffix != '.cpp':
            continue
        content = p.read_text()
        if p == main_path:
            content = main_text
        elif p.name == 'boustrophedon_explorator.cpp':
            content = instrument_bcd(content)
        elif p.name == 'room_rotator.cpp':
            content = instrument_rotation(content)
        else:
            continue
        dest = work / p.name
        dest.write_text(content)
        generated[str(p)] = str(dest)
    command = [generated.get(arg, arg) for arg in command]
    command[command.index('-o') + 1] = str(work / 'trace_native')
    command[1:1] = ['-include', str(header)]
    print('Compiling disposable trace copy', flush=True)
    result = subprocess.run(command, text=True, capture_output=True, timeout=180)
    (work / 'compile.log').write_text(result.stdout + result.stderr)
    if result.returncode:
        raise RuntimeError(result.stderr[-12000:])
    request = dict(ev['request'])
    (out / 'request.json').write_text(json.dumps(request, indent=2) + '\n')
    print('Running native decomposition and all cell sweeps', flush=True)
    result = subprocess.run([str(work / 'trace_native'), str(out / 'request.json'), str(out / 'output.json')],
                            capture_output=True, text=True, timeout=180)
    (out / 'native.log').write_text(result.stdout + result.stderr)
    result.check_returncode()
    assert json.loads((out / 'output.json').read_text()) == json.loads(original.read_text()), 'Trace changed native output'
    verification = dict(native_output_equal=True, historical_output_sha256=file_sha256(original),
        trace_output_sha256=file_sha256(out / 'output.json'),
        upstream_commit=info['upstream_commit'], original_build_identity=info['identity'],
        compile_command=shlex.join(command), trace_script_sha256=file_sha256(Path(__file__)),
        generated_sha256={str(p): file_sha256(p) for p in work.glob('*.cpp')},
        note='本次重放仅增加观测；原生输出逐值相同。最终路线使用历史保存结果，未重跑 ROS 或运动。')
    (out / 'verification.json').write_text(json.dumps(verification, indent=2, ensure_ascii=False) + '\n')
    trace = json.loads((out / 'trace.json').read_text())
    print(json.dumps(dict(cells=len(trace['sweeps']), events=len(trace['events']),
                         effective_events=sum(bool(e['cuts']) for e in trace['events']),
                         native_output_equal=True), indent=2))


if __name__ == '__main__':
    main()
