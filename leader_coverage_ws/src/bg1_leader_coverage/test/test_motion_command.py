import importlib.util
from pathlib import Path
import pytest


def module():
    path=Path(__file__).parents[1]/'scripts/motion_command.py'
    spec=importlib.util.spec_from_file_location('motion_command',path)
    m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m


@pytest.mark.parametrize('failure',['start','monitor'])
def test_abnormal_start_exit_cancels_even_if_acknowledgement_lost(failure):
    m=module();assert hasattr(m,'run_start'),'abnormal motion CLI exits do not clean up'
    calls=[]
    def call(command):
        calls.append(command)
        if command=='start' and failure=='start':raise TimeoutError('lost start response')
        return 'accepted'
    def monitor(timeout=None):
        calls.append(('monitor',timeout))
        if timeout is None:raise TimeoutError('status response lost')
    with pytest.raises(TimeoutError):m.run_start(call,monitor)
    assert calls[-2:]==['stop',('monitor',15)]


def test_rejected_duplicate_start_does_not_cancel_existing_execution():
    m=module();assert hasattr(m,'run_start')
    calls=[]
    def call(command):
        calls.append(command);raise m.CommandRejected('already executing')
    with pytest.raises(m.CommandRejected):m.run_start(call,lambda timeout=None:None)
    assert calls==['start']
