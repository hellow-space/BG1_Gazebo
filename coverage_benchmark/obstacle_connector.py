"""Assemble native swaths and turns into one audited stop-and-turn route."""
import numpy as np
from coverage_benchmark.metrics import path_length
from coverage_benchmark.swath_geometry import clip_swath
from coverage_benchmark.grid_connector import GridConnector


def usable_native_turns(ordered_swaths,path_swaths,turns):
    """Do not infer turn indices from PathComponents if native rows were merged/changed."""
    if len(turns)!=max(0,len(ordered_swaths)-1):return []
    if np.asarray(ordered_swaths).shape!=np.asarray(path_swaths).shape:return []
    if not np.array_equal(np.asarray(ordered_swaths),np.asarray(path_swaths)):return []
    return turns


def assemble_path(swaths,turns,allowed,resolution,start):
    connector=GridConnector(allowed,resolution)
    pieces=[]
    for index,swath in enumerate(swaths):
        for piece in clip_swath(swath,allowed,resolution):pieces.append((index,piece))
    if not pieces:raise ValueError('No swath remains in allowed raster')
    path=[list(map(float,start))]
    kept=0
    sections=[]
    for number,(index,piece) in enumerate(pieces):
        connection=None
        method=None
        native_verdict="not applicable after clipping or nonadjacent swaths"
        if number and index==pieces[number-1][0]+1 and index-1<len(turns):
            previous=pieces[number-1][1]
            # A native turn belongs only to the original adjacent, unchanged ends.
            if (np.allclose(previous[-1],swaths[index-1][-1],atol=1e-9,rtol=0)
                    and np.allclose(piece[0],swaths[index][0],atol=1e-9,rtol=0)
                    and len(turns[index-1])>=2):
                candidate=[path[-1],*turns[index-1],piece[0]]
                native_verdict="rejected by raster audit"
                if connector.clear(candidate):
                    connection=candidate;kept+=1;method="native_turn"
                    native_verdict="retained after full seam audit"
        if connection is None:
            calls=connector.astar_calls
            connection=connector.connect(path[-1],piece[0])
            method="astar" if connector.astar_calls>calls else "direct_checked"
        begin=len(path)-1
        path.extend(connection[1:])
        sections.append(dict(kind="connection",method=method,begin_vertex=begin,end_vertex=len(path)-1,
                             source_swath_index=index,native_turn_index=index-1 if number else None,
                             native_turn_verdict=native_verdict,start_approach=(number==0)))
        begin=len(path)-1
        path.append(piece[-1])
        sections.append(dict(kind="swath",begin_vertex=begin,end_vertex=len(path)-1,
                             source_swath_index=index,original_swath=swaths[index],clipped_swath=piece))
    if not connector.clear(path):raise ValueError('Assembled full path failed final raster audit')
    return path,dict(path_sections=sections,discarded_swath_indices=sorted(set(range(len(swaths)))-{index for index,_ in pieces}),
                     native_swaths=len(swaths),retained_swath_pieces=len(pieces),
                     native_turns_kept=kept,astar_connections=connector.astar_calls,
                     native_swath_length_m=sum(path_length(s) for s in swaths),
                     retained_swath_length_m=sum(path_length(p) for _,p in pieces),
                     start_approach_included=True,minimum_turning_radius_enforced=False)
