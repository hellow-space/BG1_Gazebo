#!/usr/bin/env python3
"""Regenerate the two BG1 worlds, reference maps and overview figures."""
import argparse
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from bg1_scenarios.build import build_scene


def preview(report,path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle,Circle
    colors={'wall':'#677680','rack':'#25546d','pallet':'#bb8d4c','column':'#859398','desk':'#bc9d70','cabinet':'#788b90'}
    fig,ax=plt.subplots(figsize=(10,8))
    for o in report['objects']:
        if o['shape']=='box':
            l,w,_=o['size'];shape=Rectangle((o['x']-l/2,o['y']-w/2),l,w)
        else:shape=Circle((o['x'],o['y']),o['radius'])
        shape.set_facecolor(colors[o['kind']]);ax.add_patch(shape)
    ax.scatter([p[0] for p in report['goals']],[p[1] for p in report['goals']],marker='.',color='#d38d13',label='Reachability probes')
    ax.scatter([0],[0],s=85,c='#168b62',marker='^',label='BG1 spawn (0, 0)')
    lx,ly=report['size'];ax.set(xlim=(-lx/2-1,lx/2+1),ylim=(-ly/2-1,ly/2+1),xlabel='x (m)',ylabel='y (m)',
        title=f'{report["title"]} — {lx:g} × {ly:g} m')
    ax.set_aspect('equal');ax.grid(alpha=.18);ax.legend(loc='upper right',fontsize=9)
    fig.tight_layout();fig.savefig(path,dpi=160);plt.close(fig)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--package',type=Path,default=Path(__file__).resolve().parents[1])
    parser.add_argument('--base-world',type=Path)
    args=parser.parse_args()
    base=args.base_world or args.package.parent/'bg1_gazebo/worlds/bg1_flat.sdf'
    (args.package/'previews').mkdir(exist_ok=True)
    for name in ('warehouse','office'):
        result=build_scene(args.package/'config'/f'{name}.yaml',base,args.package)
        preview(result,args.package/'previews'/f'{name}.png')
        print(f'{name}: {len(result["objects"])} collision objects, {len(result["goals"])} probes')
