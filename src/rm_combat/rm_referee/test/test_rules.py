from rm_referee.rules import (
    AUTO, FINISHED, MANUAL, PAUSED, RUNNING, CombatRules, RulesConfig,
)


def make_rules(**config):
    return CombatRules(RulesConfig(**config), default_mode=MANUAL)


def test_both_robots_have_independent_ammo_sequences_and_global_shot_ids():
    rules = make_rules(initial_ammo=2)
    assert rules.start(0.0)[0]
    red = rules.request_fire('red_robot', 1, 0.0)
    blue = rules.request_fire('blue_robot', 1, 0.0)
    assert red.accepted and blue.accepted
    assert red.shot_id != blue.shot_id
    assert red.target_id == 'blue_robot'
    assert blue.target_id == 'red_robot'
    assert not rules.request_fire('red_robot', 1, 1.0).accepted
    assert not rules.request_fire('blue_robot', 1, 1.0).accepted
    assert rules.participants['red_robot'].ammo == 1
    assert rules.participants['blue_robot'].ammo == 1


def test_fire_is_rejected_outside_running_and_after_reset():
    rules = make_rules()
    assert not rules.request_fire('red_robot', 1, 0.0).accepted
    assert rules.start(1.0)[0]
    assert rules.request_fire('red_robot', 2, 1.0).accepted
    rules.reset()
    assert not rules.request_fire('red_robot', 3, 2.0).accepted


def test_mode_is_per_robot_and_locked_after_start():
    rules = make_rules()
    assert rules.set_mode('red_robot', AUTO)[0]
    assert rules.participants['red_robot'].mode == AUTO
    assert rules.participants['blue_robot'].mode == MANUAL
    assert rules.start(1.0)[0]
    assert not rules.set_mode('blue_robot', AUTO)[0]


def test_projectile_damage_is_authoritative_and_clamped():
    rules = make_rules(manual_hp=20, manual_damage=20)
    rules.start(0.0)
    shot = rules.request_fire('red_robot', 10, 0.0)
    assert rules.resolve_shot(999, 'red_robot', 'blue_robot', True) is None
    result = rules.resolve_shot(shot.shot_id, 'red_robot', 'blue_robot', True)
    assert result.damage == 20 and result.remaining_hp == 0
    assert rules.phase == FINISHED
    assert rules.winner == 'red_robot'
    assert rules.resolve_shot(shot.shot_id, 'red_robot', 'blue_robot', True) is None


def test_low_speed_and_duplicate_collision_reports_do_not_damage():
    rules = make_rules()
    rules.start(0.0)
    assert rules.apply_obstacle_collision(1, 'red_robot', 0.49) == []
    assert rules.apply_obstacle_collision(1, 'red_robot', 2.0) == []
    assert rules.participants['red_robot'].hp == 500


def test_obstacle_and_robot_collision_damage():
    rules = make_rules()
    rules.start(0.0)
    obstacle = rules.apply_obstacle_collision(1, 'red_robot', 0.5)
    assert len(obstacle) == 1
    assert obstacle[0].damage == 10
    assert rules.participants['red_robot'].hp == 490
    crash = rules.apply_robot_collision(2, 'red_robot', 'blue_robot', 1.0)
    assert len(crash) == 2
    assert rules.participants['red_robot'].hp == 470
    assert rules.participants['blue_robot'].hp == 480
    assert rules.participants['red_robot'].damage_dealt == 20
    assert rules.participants['blue_robot'].damage_dealt == 20


def test_simultaneous_robot_collision_destruction_is_draw():
    rules = make_rules(manual_hp=20, robot_collision_damage=20)
    rules.start(0.0)
    rules.apply_robot_collision(1, 'red_robot', 'blue_robot', 1.0)
    assert rules.phase == FINISHED
    assert rules.winner == 'draw'
    assert rules.finish_reason == 'simultaneous destruction'


def test_timeout_uses_hp_then_damage_then_draw():
    rules = make_rules(match_duration=1.0)
    rules.start(0.0)
    rules.apply_obstacle_collision(1, 'red_robot', 1.0)
    assert rules.update_timeout(1.0)
    assert rules.winner == 'blue_robot'

    rules.reset()
    rules.start(0.0)
    rules.participants['red_robot'].damage_dealt = 30
    rules.participants['blue_robot'].damage_dealt = 20
    assert rules.update_timeout(1.0)
    assert rules.winner == 'red_robot'

    rules.reset()
    rules.start(0.0)
    assert rules.update_timeout(1.0)
    assert rules.winner == 'draw'


def test_pause_freezes_clock_and_reset_clears_all_runtime_state():
    rules = make_rules(match_duration=10.0)
    rules.set_mode('red_robot', AUTO)
    rules.start(100.0)
    assert rules.complete_checkpoint('red_robot', 101.0)
    assert rules.pause(102.0)[0]
    assert rules.phase == PAUSED
    assert rules.remaining_time(1000.0) == 8.0
    assert not rules.request_fire('red_robot', 50, 1000.0).accepted
    assert rules.resume(202.0)[0]
    assert rules.remaining_time(202.0) == 8.0
    rules.seen_collision_events.add(99)
    rules.reset()
    assert rules.phase != RUNNING
    assert not rules.pending_shots
    assert not rules.seen_collision_events
    assert rules.participants['red_robot'].damage_dealt == 0
    assert rules.participants['red_robot'].ammo == rules.config.initial_ammo
