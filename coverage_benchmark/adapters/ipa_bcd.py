"""Native IPA BCD decomposition/sweeps with audited project scheduling/connections."""
from dataclasses import replace
from pathlib import Path
from coverage_benchmark.adapters.ipa import prepare_input
from coverage_benchmark.hybrid.planner import compare
from coverage_benchmark.models import PlanningResult


def plan(request,work_dir):
    try:
        prepare_input(request)  # Validate shared rasters, dimensions, start and widths.
        result=compare(replace(request,runtime_root=Path(work_dir)),modes=('ipa_bcd',))['ipa_bcd']
        result.details['algorithm']='ipa_bcd'
        return result
    except (ValueError,RuntimeError) as error:
        details=dict(failure=True,algorithm='ipa_bcd')
        if hasattr(error,'details'):details['native_failure']=error.details
        return PlanningResult(message=str(error),details=details)
