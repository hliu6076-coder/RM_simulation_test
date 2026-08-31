from dataclasses import dataclass, field
from typing import Dict, List, Optional, Set, Tuple


PREPARING = 0
RUNNING = 1
FINISHED = 2
PAUSED = 3
AUTO = 0
MANUAL = 1
PROJECTILE = 0
OBSTACLE_COLLISION = 1
ROBOT_COLLISION = 2


@dataclass(frozen=True)
class RulesConfig:
    auto_hp: int = 500
    manual_hp: int = 500
    auto_damage: int = 20
    manual_damage: int = 20
    initial_ammo: int = 50
    fire_rate: float = 5.0
    max_range: float = 10.0
    match_duration: float = 120.0
    obstacle_damage: int = 10
    robot_collision_damage: int = 20
    min_collision_speed: float = 0.5
    checkpoint_score: int = 50
    buff_damage: int = 30
    buff_duration: float = 15.0


@dataclass
class ParticipantState:
    robot_id: str
    mode: int = MANUAL
    hp: int = 500
    ammo: int = 50
    last_fire_time: Optional[float] = None
    buff_until: float = 0.0
    checkpoint_completed: bool = False
    damage_dealt: int = 0
    bonus_score: int = 0
    seen_sequences: Set[int] = field(default_factory=set)


@dataclass(frozen=True)
class PendingShot:
    attacker_id: str
    target_id: str
    damage: int


@dataclass(frozen=True)
class FireDecision:
    accepted: bool
    reason: str
    shot_id: int = 0
    remaining_ammo: int = 0
    damage: int = 0
    target_id: str = ''


@dataclass(frozen=True)
class DamageResolution:
    event_id: int
    source: int
    attacker_id: str
    target_id: str
    damage: int
    remaining_hp: int
    relative_speed: float = 0.0


class CombatRules:
    """Authoritative symmetric two-participant combat state machine."""

    def __init__(
        self,
        config: RulesConfig,
        red_robot_id: str = 'red_robot',
        blue_robot_id: str = 'blue_robot',
        default_mode: int = MANUAL,
    ) -> None:
        if red_robot_id == blue_robot_id:
            raise ValueError('participant robot IDs must be distinct')
        self.config = config
        self.red_robot_id = red_robot_id
        self.blue_robot_id = blue_robot_id
        self.robot_ids: Tuple[str, str] = (red_robot_id, blue_robot_id)
        self.participants: Dict[str, ParticipantState] = {
            robot_id: ParticipantState(robot_id=robot_id, mode=default_mode)
            for robot_id in self.robot_ids
        }
        self.phase = PREPARING
        self.winner = ''
        self.finish_reason = ''
        self.start_time: Optional[float] = None
        self.pause_time: Optional[float] = None
        self.next_shot_id = 1
        self.pending_shots: Dict[int, PendingShot] = {}
        self.seen_collision_events: Set[int] = set()
        self.reset()

    def max_hp(self, robot_id: str) -> int:
        participant = self.participants[robot_id]
        return self.config.auto_hp if participant.mode == AUTO else self.config.manual_hp

    def opponent(self, robot_id: str) -> str:
        if robot_id == self.red_robot_id:
            return self.blue_robot_id
        if robot_id == self.blue_robot_id:
            return self.red_robot_id
        raise KeyError(robot_id)

    def reset(self) -> None:
        self.phase = PREPARING
        self.winner = ''
        self.finish_reason = ''
        self.start_time = None
        self.pause_time = None
        self.next_shot_id = 1
        self.pending_shots.clear()
        self.seen_collision_events.clear()
        for participant in self.participants.values():
            participant.hp = self.max_hp(participant.robot_id)
            participant.ammo = self.config.initial_ammo
            participant.last_fire_time = None
            participant.buff_until = 0.0
            participant.checkpoint_completed = False
            participant.damage_dealt = 0
            participant.bonus_score = 0
            participant.seen_sequences.clear()

    def set_mode(self, robot_id: str, mode: int) -> Tuple[bool, str]:
        if self.phase != PREPARING:
            return False, 'control mode is locked after match start'
        if robot_id not in self.participants:
            return False, f'unknown robot: {robot_id}'
        if mode not in (AUTO, MANUAL):
            return False, 'mode must be AUTO(0) or MANUAL(1)'
        participant = self.participants[robot_id]
        participant.mode = mode
        participant.hp = self.max_hp(robot_id)
        return True, 'control mode updated'

    def start(self, now: float) -> Tuple[bool, str]:
        if self.phase == RUNNING:
            return False, 'match is already running'
        if self.phase == FINISHED:
            return False, 'reset the match before starting again'
        if self.phase == PAUSED:
            return False, 'match is paused; use resume'
        self.phase = RUNNING
        self.start_time = now
        return True, 'match started'

    def pause(self, now: float) -> Tuple[bool, str]:
        if self.phase != RUNNING:
            return False, 'only a running match can be paused'
        self.phase = PAUSED
        self.pause_time = now
        return True, 'match paused'

    def resume(self, now: float) -> Tuple[bool, str]:
        if self.phase != PAUSED or self.pause_time is None:
            return False, 'only a paused match can be resumed'
        paused_duration = max(0.0, now - self.pause_time)
        if self.start_time is not None:
            self.start_time += paused_duration
        for participant in self.participants.values():
            if participant.last_fire_time is not None:
                participant.last_fire_time += paused_duration
            if participant.buff_until > 0.0:
                participant.buff_until += paused_duration
        self.pause_time = None
        self.phase = RUNNING
        return True, 'match resumed'

    def current_damage(self, robot_id: str, now: float) -> int:
        participant = self.participants[robot_id]
        effective_now = self.pause_time if self.phase == PAUSED else now
        if participant.mode == AUTO and effective_now is not None \
                and effective_now < participant.buff_until:
            return self.config.buff_damage
        return self.config.auto_damage if participant.mode == AUTO \
            else self.config.manual_damage

    def request_fire(self, robot_id: str, sequence: int, now: float) -> FireDecision:
        participant = self.participants.get(robot_id)
        if participant is None:
            return FireDecision(False, f'unknown robot: {robot_id}')
        if sequence in participant.seen_sequences:
            return FireDecision(
                False, 'duplicate client sequence', remaining_ammo=participant.ammo)
        participant.seen_sequences.add(sequence)
        if self.phase != RUNNING:
            return FireDecision(False, 'match is not running', remaining_ammo=participant.ammo)
        if participant.hp <= 0:
            return FireDecision(False, 'shooter is destroyed', remaining_ammo=participant.ammo)
        if participant.ammo <= 0:
            return FireDecision(False, 'out of ammunition', remaining_ammo=0)
        interval = 1.0 / self.config.fire_rate
        if participant.last_fire_time is not None \
                and now - participant.last_fire_time + 1e-9 < interval:
            return FireDecision(False, 'fire-rate limit', remaining_ammo=participant.ammo)
        shot_id = self.next_shot_id
        self.next_shot_id += 1
        participant.last_fire_time = now
        participant.ammo -= 1
        target_id = self.opponent(robot_id)
        damage = self.current_damage(robot_id, now)
        self.pending_shots[shot_id] = PendingShot(robot_id, target_id, damage)
        return FireDecision(
            True, 'accepted', shot_id, participant.ammo, damage, target_id)

    def resolve_shot(
        self, shot_id: int, shooter_id: str, target_id: str, hit: bool
    ) -> Optional[DamageResolution]:
        pending = self.pending_shots.pop(shot_id, None)
        if pending is None or self.phase != RUNNING:
            return None
        if pending.attacker_id != shooter_id or pending.target_id != target_id or not hit:
            return None
        return self._apply_single_damage(
            shot_id, PROJECTILE, shooter_id, target_id, pending.damage)

    def apply_obstacle_collision(
        self, event_id: int, robot_id: str, relative_speed: float
    ) -> List[DamageResolution]:
        if not self._accept_collision(event_id, relative_speed):
            return []
        if robot_id not in self.participants:
            return []
        resolution = self._apply_single_damage(
            event_id, OBSTACLE_COLLISION, '', robot_id,
            self.config.obstacle_damage, relative_speed)
        return [resolution] if resolution is not None else []

    def apply_robot_collision(
        self, event_id: int, robot_a: str, robot_b: str, relative_speed: float
    ) -> List[DamageResolution]:
        if not self._accept_collision(event_id, relative_speed):
            return []
        if robot_a not in self.participants or robot_b not in self.participants \
                or robot_a == robot_b:
            return []
        damage = self.config.robot_collision_damage
        state_a = self.participants[robot_a]
        state_b = self.participants[robot_b]
        applied_a = min(damage, state_a.hp)
        applied_b = min(damage, state_b.hp)
        state_a.hp = max(0, state_a.hp - damage)
        state_b.hp = max(0, state_b.hp - damage)
        state_b.damage_dealt += applied_a
        state_a.damage_dealt += applied_b
        resolutions = [
            DamageResolution(
                event_id, ROBOT_COLLISION, robot_b, robot_a,
                applied_a, state_a.hp, relative_speed),
            DamageResolution(
                event_id, ROBOT_COLLISION, robot_a, robot_b,
                applied_b, state_b.hp, relative_speed),
        ]
        self._finish_after_damage()
        return resolutions

    def _accept_collision(self, event_id: int, relative_speed: float) -> bool:
        if event_id in self.seen_collision_events:
            return False
        self.seen_collision_events.add(event_id)
        return self.phase == RUNNING and relative_speed >= self.config.min_collision_speed

    def _apply_single_damage(
        self,
        event_id: int,
        source: int,
        attacker_id: str,
        target_id: str,
        damage: int,
        relative_speed: float = 0.0,
    ) -> Optional[DamageResolution]:
        if self.phase != RUNNING or target_id not in self.participants:
            return None
        target = self.participants[target_id]
        applied = min(max(0, damage), target.hp)
        target.hp = max(0, target.hp - damage)
        if attacker_id in self.participants and attacker_id != target_id:
            self.participants[attacker_id].damage_dealt += applied
        resolution = DamageResolution(
            event_id, source, attacker_id, target_id, applied,
            target.hp, relative_speed)
        self._finish_after_damage()
        return resolution

    def _finish_after_damage(self) -> None:
        red_dead = self.participants[self.red_robot_id].hp <= 0
        blue_dead = self.participants[self.blue_robot_id].hp <= 0
        if not red_dead and not blue_dead:
            return
        self.phase = FINISHED
        if red_dead and blue_dead:
            self.winner = 'draw'
            self.finish_reason = 'simultaneous destruction'
        elif red_dead:
            self.winner = self.blue_robot_id
            self.finish_reason = f'{self.red_robot_id} destroyed'
        else:
            self.winner = self.red_robot_id
            self.finish_reason = f'{self.blue_robot_id} destroyed'

    def complete_checkpoint(self, robot_id: str, now: float) -> bool:
        participant = self.participants.get(robot_id)
        if participant is None or self.phase != RUNNING \
                or participant.mode != AUTO or participant.checkpoint_completed:
            return False
        participant.checkpoint_completed = True
        participant.bonus_score += self.config.checkpoint_score
        participant.buff_until = now + self.config.buff_duration
        return True

    def update_timeout(self, now: float) -> bool:
        if self.phase != RUNNING or self.start_time is None:
            return False
        if now - self.start_time < self.config.match_duration:
            return False
        self.phase = FINISHED
        red = self.participants[self.red_robot_id]
        blue = self.participants[self.blue_robot_id]
        if red.hp != blue.hp:
            self.winner = self.red_robot_id if red.hp > blue.hp else self.blue_robot_id
            self.finish_reason = 'time expired: remaining HP'
        elif red.damage_dealt != blue.damage_dealt:
            self.winner = self.red_robot_id \
                if red.damage_dealt > blue.damage_dealt else self.blue_robot_id
            self.finish_reason = 'time expired: damage dealt'
        else:
            self.winner = 'draw'
            self.finish_reason = 'time expired: draw'
        return True

    def remaining_time(self, now: float) -> float:
        if self.start_time is None:
            return self.config.match_duration
        effective_now = self.pause_time if self.phase == PAUSED else now
        return max(0.0, self.config.match_duration - (effective_now - self.start_time))

    def buff_remaining(self, robot_id: str, now: float) -> float:
        participant = self.participants[robot_id]
        effective_now = self.pause_time if self.phase == PAUSED else now
        return max(0.0, participant.buff_until - effective_now)

    def score(self, robot_id: str) -> int:
        participant = self.participants[robot_id]
        return participant.damage_dealt + participant.bonus_score
