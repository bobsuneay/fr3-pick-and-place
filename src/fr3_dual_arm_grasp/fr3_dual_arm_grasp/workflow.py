"""Deterministic, cancellable recipe shared by GUI, services and offline tests."""
import time
from .teach_model import recipe


def run_workflow(app):
    app.initialize_scene()
    app.book.validate_complete()
    app.motion.guard(True)
    if app.scene.owner or app.recovery_required:
        raise RuntimeError('Recover previous run before starting a new demo')
    app.scene.place_initial(app.grasp_target())
    # Once a run begins, failure must never automatically restart at the first step.
    app.recovery_required = True
    steps = list(recipe())
    for index, (kind, side, key) in enumerate(steps, 1):
        app.motion.guard(True)
        app.publish(f'{index}/{len(steps)} {kind} {side} {key}')
        try:
            _run_step(app, kind, side, key)
        except Exception as exc:
            app._wait_skip_or_stop(f'{index}/{len(steps)} {kind} {side} {key}', exc)
    app.scene.allow([], table=True)
    app.recovery_required = False
    app.publish('完成：右手抓取→向头部相机多角度展示→左手接取→放下')


def _run_step(app, kind, side, key):
    if kind in ('move', 'view', 'orient', 'approach', 'lift'):
        if key == 'left_place':
            # Workpiece/table contact is intentional only for placement.
            app.scene.allow(['left'], table=True)
        # Orient in joint space first, then descend straight with MoveL.  The
        # orientation-only and pure-translation moves both complete reliably.
        app.move_point(key, side, execute=True, linear=(kind == 'approach'))
        if kind == 'view':
            if app.stop_event.wait(app.dwell):
                raise RuntimeError('Cancelled during display')
    elif kind == 'scan':
        app.scan_display(side)
    elif kind == 'receive':
        app.move_handover_receive()
    elif kind == 'retreat':
        app.retreat_donor()
    elif kind in ('preplace', 'place'):
        if kind == 'place':
            app.scene.allow(['left'], table=True)
        app.place_move(above=(kind == 'preplace'))
    elif kind == 'grip':
        app.motion.gripper(side, app.book.points[key][side]['gap_m'], execute=True)
    elif kind == 'grasp':
        if side == 'left':
            app.verify_handover_alignment()
        # Command the physical closed endpoint; the gripper's internal
        # force limit stops on the part and ROS accepts the stable stall.
        app.motion.gripper(side, 0.0, execute=True)
        app.verify_grasp(side)
    elif kind in ('attach', 'transfer'):
        app.scene.attach(side, transfer=(kind == 'transfer'))
    elif kind == 'touch':
        app.scene.set_touch(['left', 'right'])
    elif kind == 'touch_only':
        app.scene.set_touch([side])
    elif kind == 'detach':
        app.scene.detach()
