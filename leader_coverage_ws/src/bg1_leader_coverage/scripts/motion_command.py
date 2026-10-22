#!/usr/bin/env python3
"""Foreground start/monitor; Ctrl+C cancels and waits for termination."""
import argparse
import json
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_srvs.srv import Trigger
import time
import signal


class CommandRejected(RuntimeError):
    pass


def stop_and_wait(call,monitor):
    try:print(call('stop'),flush=True)
    except Exception as error:print(f'Stop request error: {error}; checking final state.',flush=True)
    monitor(15)


def run_start(call,monitor):
    try:
        try:print(call('start'),flush=True)
        except CommandRejected:raise
        except (Exception,KeyboardInterrupt):
            stop_and_wait(call,monitor)
            raise
    except CommandRejected:
        raise
    try:monitor()
    except KeyboardInterrupt:stop_and_wait(call,monitor)
    except Exception:
        stop_and_wait(call,monitor)
        raise


def main():
    parser=argparse.ArgumentParser();parser.add_argument('command',choices=['start','stop','status'])
    args=parser.parse_args();rclpy.init(signal_handler_options=SignalHandlerOptions.NO);node=Node('leader_motion_command')
    clients={command:node.create_client(Trigger,f'/leader_coverage/motion/{command}') for command in ('start','stop','status')}
    def call(command):
        client=clients[command]
        if not client.wait_for_service(timeout_sec=5):raise RuntimeError('Motion service unavailable; rebuild/restart leader_coverage_ws/run.sh')
        future=client.call_async(Trigger.Request());rclpy.spin_until_future_complete(node,future,timeout_sec=5)
        if not future.done():raise RuntimeError('Motion command timed out')
        response=future.result()
        if not response.success:raise CommandRejected(response.message)
        return response.message
    def monitor(timeout=None):
        deadline=time.monotonic()+timeout if timeout else float('inf');previous=None
        while True:
            state=json.loads(call('status'))
            if (state['phase'],state['message'])!=previous:
                print(f"{state['phase']}: {state['message']}",flush=True);previous=(state['phase'],state['message'])
            if state['phase'] not in ('approaching','following','stopping'):
                if state['phase']=='failed':raise RuntimeError(state['message'])
                return
            if time.monotonic()>deadline:raise RuntimeError('Cancellation not confirmed; stop the simulation.')
            rclpy.spin_once(node,timeout_sec=.5)
    def interrupt(signum,frame):raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,interrupt)
    try:
        if args.command=='start':run_start(call,monitor)
        elif args.command=='stop':stop_and_wait(call,monitor)
        else:print(call('status'),flush=True)
    finally:node.destroy_node();rclpy.try_shutdown()

if __name__=='__main__':main()
