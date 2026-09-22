"""Deterministic, cancellable recipe shared by GUI, services and offline tests."""
import time
from .teach_model import recipe


def run_workflow(app):
    app.initialize_scene()
    app.book.validate_complete()
    app.motion.guard(True)
    if app.scene.owner or app.recovery_required:
        raise RuntimeError('Recover previous run before starting a new demo')
    app.scene.place_initial(app.book.points['right_grasp']['right']['tcp'])
    # Once a run begins, failure must never automatically restart at the first step.
    app.recovery_required = True
    for index, (kind, side, key) in enumerate(recipe(), 1):
        app.motion.guard(True)
        app.publish(f'{index}/{len(recipe())} {kind} {side} {key}')
        if kind in ('move', 'view', 'approach', 'lift'):
            if key == 'left_place':
                # Workpiece/table contact is intentional only for placement.
                app.scene.allow(['left'], table=True)
            app.move_point(
                key, side, execute=True,
                linear=kind in ('approach', 'lift'))
            if kind == 'view':
                if app.stop_event.wait(app.dwell):
                    raise RuntimeError('Cancelled during display')
        elif kind == 'scan':
            app.scan_display(side)
        elif kind == 'retreat':
            app.retreat_donor()
        elif kind in ('preplace', 'place'):
            if kind == 'place':
                app.scene.allow(['left'], table=True)
            app.place_move(above=(kind == 'preplace'))
        elif kind == 'grip':
            app.motion.gripper(side, app.book.points[key][side]['gap_m'], execute=True)
        elif kind == 'confirm':
            app.confirm_event.clear()
            app.awaiting_confirmation = key
            app.publish(key)
            try:
                while not app.confirm_event.wait(0.1):
                    app.motion.guard(True)
            finally:
                app.awaiting_confirmation = ''
        elif kind in ('attach', 'transfer'):
            app.scene.attach(side, transfer=(kind == 'transfer'))
        elif kind == 'touch':
            app.scene.set_touch(['left', 'right'])
        elif kind == 'touch_only':
            app.scene.set_touch([side])
        elif kind == 'detach':
            app.scene.detach()
    app.scene.allow([], table=True)
    app.recovery_required = False
    app.publish('完成：右手抓取→展示→左手交接→展示→放下')
