"""
R2 Meihua Forest Full Planner (state-aware, with R1 coordination)
=================================================================
Takes the COMPLETE Forest state -- 3 R1 KFS blocks, 4 R2 KFS blocks, 1 Fake
KFS block (remaining 4 blocks are empty) -- and produces R2's movement plan,
OR, when R2 has no legal path, the minimal set of R1 KFS blocks that R1 must
clear to open one.

Why R1 matters now (rule-driven):
- Rule 8.10: R2 commits a violation if it MOVES ONTO a block that still has
  a KFS on it. So every block R2 physically stands on must be empty at the
  moment it steps there -- unless R2 is COLLECTING that KFS as it climbs on
  (see the pick model below).
- Rule 4.4.16 + 8.8 + 8.9: R2 may only collect R2 KFS. R2 touching an R1 KFS
  (outside the Arena) or the Fake KFS is a violation. So R2 CANNOT clear R1
  KFS blocks or the Fake block to make a path.
- R2 holding capacity = 2 (physical). R2 collects exactly its 2 scoring R2
  KFS, which are also the only 2 blocks R2 can empty/clear for itself.
- Therefore, for R2's foot-path:
    PASSABLE  = empty blocks
              + the 2 R2 KFS blocks R2 collects (emptied on pickup)
              + any R1 KFS blocks that R1 has cleared (rule 4.4.1: R1 picks
                R1 KFS from the Pathway)
    OBSTACLE  = Fake block (always)
              + the 2 R2 KFS blocks R2 does NOT collect
              + R1 KFS blocks R1 has NOT cleared
- This is exactly how the exit can be "blocked": if obstacles wall off both
  tier-A exit blocks (10 and 12), R2 is stuck. R1 clearing its own KFS off
  the right blocks can re-open a route. If the wall is made only of Fake +
  uncollected R2 KFS (which neither robot may move), the layout is genuinely
  unsolvable and the tool says so.

R1 holding capacity = 2 (physical), but rule 4.4.3 lets R1 loop the Pathway
as many times as needed, so R1 can ultimately clear all 3 of its KFS over
multiple trips. The tool reports the minimal clear-set and flags if it needs
more than one R1 trip.

PICK MODEL (per UPDATE.md Layer A -- replaces the old "stationary reach"):
- A pick is a PARTIAL CLIMB onto the target block, performed from an
  adjacent block exactly ONE TIER away: align, extend pneumatics, drive
  halfway onto the target grid, pick, then EITHER complete the climb onto
  the block OR reverse back to the previous block (R2's choice -- reversing
  preserves routing flexibility). Both outcomes are searched.
- All in-forest adjacencies are exactly one tier, so interior picks are
  always mechanically valid. From the Pathway (ground) only block 2 (tier A,
  200mm) is one tier up; blocks 1 and 3 (tier B, 400mm) are a two-tier reach
  and CANNOT be picked from the Pathway. No entrance-zone strafing exists.
- R2 KFS on block 1 or 3 must be picked from INSIDE the forest, from a
  one-tier neighbor (block 2, 4, or 6).
- Rule 4.4.14 is NOT enforced (dropped per UPDATE.md A3). If block 2 holds
  an R2 KFS, R2 must pick it to enter anyway (it is the only entrance), so
  that case orders itself via passability; no other first-pick forcing.
- Downward picks: the choreography for reversing back UP mid-descent is
  undefined on the bridge, so a downward pick is only generated as
  "complete onto the block" (never pick-then-reverse).

Grid (display orientation, entrance row at bottom):
    10  11  12
    7   8   9
    4   5   6
    1   2   3

Heights (right-side team, A=200 B=400 C=600):
    1:B 2:A 3:B  4:C 5:B 6:A  7:B 8:C 9:B  10:A 11:B 12:A

TOGGLES:
[T1] NO_DOWNWARD_PICK (default False): when True, R2 may not pick a KFS on
     a block one tier BELOW its current block -- every KFS must be collected
     via an UPWARD approach. Picking during a descent is the mechanically
     risky maneuver; this mode avoids it for bench testing. Some layouts get
     longer paths / more R1 clearing or become impossible -- that is
     intended and reported via the normal status.
[T2] dev_free_path_actions(): dev-only mode that compiles an operator-given
     ordered block route into CLIMB_UP/CLIMB_DOWN (+ROTATE) primitives with
     NO picks and NO rule checks. Separate entry point from plan() so it
     cannot be used for a real run by accident.

FLAGGED RULE INTERPRETATIONS (please confirm -- they affect results):
[I3] Entry is via block 2 only (sole tier-A entrance block); exits are 10 or
     12 only (sole tier-A exit blocks; 11 is tier B -> 2-tier drop disallowed).
[I4] Fake block treated as fully impassable; never stepped on.
"""

from collections import deque
from itertools import combinations

# ---------------------------------------------------------------------------
# Config toggles
# ---------------------------------------------------------------------------
NO_DOWNWARD_PICK = False             # [T1] forbid picks one tier below
R2_CAPACITY = 2
R1_CAPACITY = 2
WANT_COLLECT = 2                     # R2 target number of R2 KFS to collect

# ---------------------------------------------------------------------------
# Field constants
# ---------------------------------------------------------------------------
GRID_COLS = 3
ALL_BLOCKS = list(range(1, 13))
ENTRANCE = 2
VALID_EXITS_RANKED = [12, 10]        # 12 preferred (closer to Ramp)
START = 0                            # virtual Pathway / entrance-zone node
ENTRANCE_ROW = {1, 2, 3}

HEIGHTS = {
    0: 'GROUND',
    1: 'B', 2: 'A', 3: 'B',
    4: 'C', 5: 'B', 6: 'A',
    7: 'B', 8: 'C', 9: 'B',
    10: 'A', 11: 'B', 12: 'A',
}
TIER_LEVEL = {'GROUND': 0, 'A': 1, 'B': 2, 'C': 3}
TIER_MM = {'GROUND': 0, 'A': 200, 'B': 400, 'C': 600}
COMPASS = ['NORTH', 'EAST', 'SOUTH', 'WEST']
DISPLAY_ROWS = [[10, 11, 12], [7, 8, 9], [4, 5, 6], [1, 2, 3]]


# ---------------------------------------------------------------------------
# Geometry
# ---------------------------------------------------------------------------
def grid_neighbors(b):
    row, col = divmod(b - 1, GRID_COLS)
    out = []
    if row > 0:
        out.append(b - GRID_COLS)
    if row < 3:
        out.append(b + GRID_COLS)
    if col > 0:
        out.append(b - 1)
    if col < GRID_COLS - 1:
        out.append(b + 1)
    return out


def tier(b):
    return TIER_LEVEL[HEIGHTS[b]]


def move_neighbors(b):
    """Blocks R2 can CLIMB to from b (must also be passable, checked later)."""
    if b == START:
        return [ENTRANCE]          # can only climb in via block 2 (tier A)
    return grid_neighbors(b)


def pick_neighbors(b):
    """Blocks R2 can PICK from position b (a pick is a partial one-tier
    climb, so the pickable set equals the climbable set: from the Pathway
    only block 2 is one tier up; blocks 1/3 are two tiers and unreachable)."""
    return move_neighbors(b)


def direction_of(a, b):
    if a == START:
        return 'NORTH'
    ra, ca = divmod(a - 1, GRID_COLS)
    rb, cb = divmod(b - 1, GRID_COLS)
    dr, dc = rb - ra, cb - ca
    return {(1, 0): 'NORTH', (-1, 0): 'SOUTH',
            (0, 1): 'EAST', (0, -1): 'WEST'}[(dr, dc)]


def rotation_needed(cur, tgt):
    return ((COMPASS.index(tgt) - COMPASS.index(cur)) % 4) * 90


def block_label(b):
    if b == START:
        return 'Pathway (ground)'
    return f'block {b} ({HEIGHTS[b]}, {TIER_MM[HEIGHTS[b]]}mm)'


# ---------------------------------------------------------------------------
# Core R2 search for a FIXED set of cleared R1 blocks
# ---------------------------------------------------------------------------
def r2_search(r1_blocks, r2_blocks, fake_block, cleared_r1, want=WANT_COLLECT,
              no_downward_pick=NO_DOWNWARD_PICK):
    """
    Find shortest legal R2 plan given which R1 blocks are cleared.
    Returns dict(positions, events, collected, exit_block, steps) or None.

    State = (position, frozenset(collected_blocks)).
    Events (the maneuver sequence, implicit start at START):
        ('MOVE', b)          -- climb up/down onto empty/cleared block b
        ('PICK', t, end_on)  -- partial-climb pick of the R2 KFS on t;
                                end_on=True  -> completes the climb onto t
                                end_on=False -> reverses back, stays put
    A block is foot-passable iff:
        - it's empty (no scroll), OR
        - it's an R2 KFS block already in `collected` (emptied on pickup), OR
        - it's an R1 block in `cleared_r1`.
    Stepping onto an uncollected R2 KFS block is only possible AS the pick
    that collects it (rule 8.10), which is what ('PICK', t, True) encodes.
    """
    r1_set = set(r1_blocks)
    r2_set = set(r2_blocks)
    uncleared_r1 = r1_set - set(cleared_r1)

    def passable(block, collected):
        if block == fake_block:
            return False
        if block in uncleared_r1:
            return False
        if block in r2_set and block not in collected:
            return False
        return True

    best = None  # (steps, ramp_rank, events, exit_block)

    for exit_block in VALID_EXITS_RANKED:
        if not passable(exit_block, frozenset(r2_set)):
            # exit permanently blocked (Fake/R1-uncleared on it); collected
            # can't help unless exit itself is an R2 KFS we collect -> handled
            # by the search below, so don't prematurely skip R2-KFS exits.
            if exit_block not in r2_set:
                continue

        start_state = (START, frozenset())
        visited = {start_state}
        queue = deque([(start_state, [])])
        found = None

        while queue:
            (pos, collected), events = queue.popleft()

            if pos == exit_block and len(collected) == want:
                found = events
                break

            # --- Option 1: pick an adjacent one-tier R2 KFS (partial climb) ---
            if len(collected) < want:
                for tgt in pick_neighbors(pos):
                    if tgt not in r2_set or tgt in collected:
                        continue
                    downward = tier(tgt) < tier(pos)
                    if no_downward_pick and downward:
                        continue
                    new_collected = collected | {tgt}
                    # (a) complete the climb onto the picked block
                    state = (tgt, new_collected)
                    if state not in visited:
                        visited.add(state)
                        queue.append((state, events + [('PICK', tgt, True)]))
                    # (b) reverse back after picking (upward picks only: the
                    # bridge has no choreography for backing UP mid-descent)
                    if not downward:
                        state = (pos, new_collected)
                        if state not in visited:
                            visited.add(state)
                            queue.append((state, events + [('PICK', tgt, False)]))

            # --- Option 2: climb to an adjacent passable block ---
            for nxt in move_neighbors(pos):
                if not passable(nxt, collected):
                    continue
                state = (nxt, collected)
                if state not in visited:
                    visited.add(state)
                    queue.append((state, events + [('MOVE', nxt)]))

        if found is None:
            continue

        steps = len(found)                     # number of maneuvers
        ramp_rank = VALID_EXITS_RANKED.index(exit_block)
        key = (steps, ramp_rank)
        if best is None or key < (best[0], best[1]):
            best = (steps, ramp_rank, found, exit_block)

    if best is None:
        return None

    steps, ramp_rank, events, exit_block = best
    positions = [START]
    collected_order = []
    for event in events:
        if event[0] == 'MOVE':
            positions.append(event[1])
        else:
            _, tgt, end_on = event
            collected_order.append(tgt)
            if end_on:
                positions.append(tgt)
    return {
        'positions': positions,          # blocks stood on, START at front
        'events': events,                # maneuver list, see docstring
        'collected': collected_order,
        'exit_block': exit_block,
        'steps': steps,
    }


# ---------------------------------------------------------------------------
# Top-level planner
# ---------------------------------------------------------------------------
def plan(r1_blocks, r2_blocks, fake_block, want=WANT_COLLECT,
         no_downward_pick=NO_DOWNWARD_PICK):
    """
    Returns a dict describing the outcome:
      status: 'ok_no_clear' | 'ok_needs_clear' | 'impossible'
      route: the r2_search result (or None)
      clear_set: R1 blocks R1 must clear, ORDERED by when R2 reaches them
      want_used: how many R2 KFS R2 collects in the returned plan

    Objective (reflecting that R1 collects 2 R1 KFS anyway, so clearing up to
    2 is free for the team):
      1. collect as many R2 KFS as possible (target `want`, then fall back)
      2. avoid forcing R1 a SECOND Pathway trip (i.e. avoid needing all 3
         R1 blocks cleared, since R1 capacity is 2)
      3. shortest R2 path
      4. fewest R1 blocks actually traversed (more freedom for R1's choice)
      5. prefer exit 12 (closer to Ramp)
    """
    r1_set = list(r1_blocks)

    for target in range(want, 0, -1):
        best = None  # (score, route, actual_clears_ordered)

        for size in range(0, len(r1_set) + 1):
            for clear_combo in combinations(r1_set, size):
                route = r2_search(r1_blocks, r2_blocks, fake_block,
                                  cleared_r1=set(clear_combo), want=target,
                                  no_downward_pick=no_downward_pick)
                if route is None:
                    continue
                # Which R1 blocks does this path ACTUALLY step on?
                pos_list = route['positions']
                actual = [b for b in pos_list if b in set(r1_set)]
                order_idx = {b: i for i, b in enumerate(pos_list)}
                actual_ordered = sorted(set(actual), key=lambda b: order_idx[b])
                n_clear = len(actual_ordered)
                extra_trip = 1 if n_clear > R1_CAPACITY else 0
                exit_rank = 0 if route['exit_block'] == 12 else 1
                score = (extra_trip, route['steps'], n_clear, exit_rank)
                if best is None or score < best[0]:
                    best = (score, route, actual_ordered)

        if best is not None:
            score, route, actual_ordered = best
            status = 'ok_no_clear' if not actual_ordered else 'ok_needs_clear'
            return {
                'status': status,
                'route': route,
                'clear_set': actual_ordered,        # ordered by R2 traversal
                'want_used': target,
                'fallback_collect': target < want,
                'needs_extra_r1_trip': score[0] == 1,
            }

    return {
        'status': 'impossible',
        'route': None,
        'clear_set': [],
        'want_used': 0,
        'fallback_collect': True,
        'needs_extra_r1_trip': False,
    }


# ---------------------------------------------------------------------------
# Action sequence generation
# ---------------------------------------------------------------------------
def generate_actions(route):
    """Compile a planned route into executor primitives.

    Emits FORWARD_INIT, ROTATE_90/180/270, CLIMB_UP/CLIMB_DOWN, and
    PICK_BLOCK_UP/PICK_BLOCK_DOWN. No VISUAL_SERVO_BLOCK: the KFS visual
    servo is dropped from the flow for now (UPDATE.md A6) -- picks are
    positioned purely by the climb-on drive (odometry/IR). Pick meta carries
    end_on: True = complete the climb onto the picked block, False = reverse
    back to the block the pick was made from.
    """
    events = route['events']
    exit_block = route['exit_block']
    actions = []
    facing = 'NORTH'
    pos = START

    def emit(a, c=None, **meta):
        # meta carries structured params (target block, height, end_on) so
        # the executor never has to parse them back out of the comment.
        actions.append((a, c, meta))

    def face(direction, desc):
        nonlocal facing
        rot = rotation_needed(facing, direction)
        if rot:
            emit(f'ROTATE_{rot}', f'turn to face {desc}')
            facing = direction

    emit('FORWARD_INIT',
         'approach from 1m before block 2, waiting for R1 to clear the Forest')

    for event in events:
        if event[0] == 'MOVE':
            b = event[1]
            face(direction_of(pos, b), f'block {b}')
            if tier(b) > tier(pos):
                emit('CLIMB_UP', f'{block_label(pos)} -> {block_label(b)}')
            else:
                emit('CLIMB_DOWN', f'{block_label(pos)} -> {block_label(b)}')
            pos = b
        else:
            _, tgt, end_on = event
            face(direction_of(pos, tgt), f'block {tgt}')
            upward = tier(tgt) > tier(pos)
            finish = ('complete climb onto it' if end_on
                      else f'reverse back to {block_label(pos)}')
            emit('PICK_BLOCK_UP' if upward else 'PICK_BLOCK_DOWN',
                 f'partial-climb pick of R2 KFS at {block_label(tgt)}, '
                 f'then {finish}',
                 block=tgt, height=HEIGHTS[tgt], end_on=end_on)
            if end_on:
                pos = tgt

    emit('CLIMB_DOWN',
         f'{block_label(exit_block)} -> Pathway (ground), EXIT Forest')
    return actions


# ---------------------------------------------------------------------------
# Dev / free-path mode (UPDATE.md A5) -- NOT for competition runs
# ---------------------------------------------------------------------------
def dev_free_path_actions(blocks, descend_exit=False):
    """Compile an operator-specified block route into climb primitives.

    Bench-test mode: walks the given ordered block list, ignoring KFS/Fake/R1
    occupancy and every passability rule. Emits ONLY ROTATE_* and
    CLIMB_UP/CLIMB_DOWN -- no picks. The route starts with R2 on the Pathway
    facing the first block; if descend_exit is True a final CLIMB_DOWN off
    the last block back to the Pathway is appended.

    Consecutive blocks must be grid-adjacent (otherwise the primitives are
    meaningless); tier rules are NOT checked beyond a warning comment when
    the first block is not tier A (a >1-tier climb from the ground is not
    mechanically possible -- the operator is trusted in this mode).
    """
    if not blocks:
        raise ValueError('dev_free_path: route is empty')
    for b in blocks:
        if not isinstance(b, int) or not 1 <= b <= 12:
            raise ValueError(f'dev_free_path: invalid block {b!r}')
    for a, b in zip(blocks, blocks[1:]):
        if b not in grid_neighbors(a):
            raise ValueError(
                f'dev_free_path: blocks {a} and {b} are not adjacent')

    actions = []
    facing = 'NORTH'
    pos = START

    def emit(a, c=None, **meta):
        actions.append((a, c, meta))

    def face(direction, desc):
        nonlocal facing
        rot = rotation_needed(facing, direction)
        if rot:
            emit(f'ROTATE_{rot}', f'turn to face {desc}')
            facing = direction

    first = blocks[0]
    warn = ('' if HEIGHTS[first] == 'A'
            else f' [WARNING: {TIER_MM[HEIGHTS[first]]}mm is >1 tier from ground]')
    emit('CLIMB_UP', f'{block_label(pos)} -> {block_label(first)}{warn}')
    pos = first

    for b in blocks[1:]:
        face(direction_of(pos, b), f'block {b}')
        if tier(b) > tier(pos):
            emit('CLIMB_UP', f'{block_label(pos)} -> {block_label(b)}')
        else:
            emit('CLIMB_DOWN', f'{block_label(pos)} -> {block_label(b)}')
        pos = b

    if descend_exit:
        warn = ('' if HEIGHTS[pos] == 'A'
                else f' [WARNING: {TIER_MM[HEIGHTS[pos]]}mm is >1 tier to ground]')
        emit('CLIMB_DOWN', f'{block_label(pos)} -> Pathway (ground){warn}')

    return actions


# ---------------------------------------------------------------------------
# Display
# ---------------------------------------------------------------------------
def classify(b, r1, r2, fake):
    if b == fake:
        return 'FAKE'
    if b in r1:
        return 'R1'
    if b in r2:
        return 'R2'
    return '.'


def print_grid(r1, r2, fake, route=None, clear_set=None):
    clear_set = set(clear_set or [])
    foot = set(p for p in (route['positions'] if route else []) if p != START)
    print()
    for row in DISPLAY_ROWS:
        cells = []
        for b in row:
            kind = classify(b, r1, r2, fake)
            mark = ''
            if b in foot:
                mark = '*'
            if b in clear_set:
                kind = 'R1>X'   # R1 KFS that must be cleared
            tag = f'{b:>2}{HEIGHTS[b]}{mark}{kind:<4}'
            cells.append(f'[{tag}]')
        print('  '.join(cells))
    print()


def parse_blocks(prompt, count, taken, boundary_only=False):
    interior = {5, 8}
    while True:
        raw = input(prompt).strip()
        try:
            blocks = [int(x) for x in raw.replace(',', ' ').split()]
        except ValueError:
            print('  Parse error, try again.')
            continue
        if len(blocks) != count:
            print(f'  Expected {count} blocks, got {len(blocks)}.')
            continue
        if len(set(blocks)) != count:
            print('  Blocks must be distinct.')
            continue
        if any(b < 1 or b > 12 for b in blocks):
            print('  Blocks must be 1..12.')
            continue
        if boundary_only and (set(blocks) & interior):
            bad = sorted(set(blocks) & interior)
            print(f'  R1 KFS must be on boundary blocks adjacent to the Pathway '
                  f'(rule 3.3.3). Blocks {bad} are interior and not allowed.')
            continue
        if any(b in taken for b in blocks):
            print(f'  Overlaps an already-assigned block: {sorted(set(blocks) & taken)}.')
            continue
        return blocks


def parse_single(prompt, taken):
    while True:
        raw = input(prompt).strip()
        try:
            b = int(raw)
        except ValueError:
            print('  Parse error, try again.')
            continue
        if b < 1 or b > 12:
            print('  Block must be 1..12.')
            continue
        if b in ENTRANCE_ROW:
            print('  Fake KFS cannot be on entrance blocks 1,2,3 (rule 4.1.4).')
            continue
        if b in taken:
            print('  Overlaps an already-assigned block.')
            continue
        return b


def report(r1, r2, fake, no_downward_pick=NO_DOWNWARD_PICK):
    result = plan(r1, r2, fake, no_downward_pick=no_downward_pick)
    print()
    print(f'R1 KFS : {sorted(r1)}')
    print(f'R2 KFS : {sorted(r2)}')
    print(f'Fake   : {fake}')
    empties = [b for b in ALL_BLOCKS if b not in set(r1) | set(r2) | {fake}]
    print(f'Empty  : {empties}')
    if no_downward_pick:
        print('MODE   : no_downward_pick (every KFS via an upward approach)')

    route = result['route']
    clear_set = result['clear_set']

    if result['status'] == 'impossible':
        print_grid(r1, r2, fake)
        print('RESULT: NO LEGAL PATH, even if R1 clears all of its KFS.')
        if no_downward_pick:
            print('NOTE: no_downward_pick is ON; the layout may be solvable')
            print('with downward picks allowed.')
        print('The exit is walled off by obstacles R2 may not move (Fake KFS')
        print('and/or uncollected R2 KFS). No R1 action can fix this.')
        return

    print_grid(r1, r2, fake, route=route, clear_set=clear_set)

    if result['fallback_collect']:
        print(f"NOTE: collecting {WANT_COLLECT} was impossible; plan collects "
              f"{result['want_used']} (legal minimum is 1, rule 4.4.17).")

    if result['status'] == 'ok_no_clear':
        print('R1 COORDINATION: not required. R2 has a legal path with all 3')
        print('R1 KFS left in place. R1 is free to collect whichever 2 R1 KFS')
        print(f'it wants (any of {sorted(r1)}) for the Arena.')
    else:
        ordered = clear_set  # already ordered by R2 traversal
        if len(ordered) == 1:
            seq = f'block {ordered[0]}'
        else:
            seq = ', then '.join(f'block {b}' for b in ordered)
        print('R1 COORDINATION REQUIRED. R1 must clear these R1 KFS to open')
        print(f'R2\'s path, in this order (so R1 stays ahead of R2): {seq}')
        if result['needs_extra_r1_trip']:
            print(f'  WARNING: {len(ordered)} blocks needed > R1 capacity '
                  f'{R1_CAPACITY} -> R1 must make a 2nd Pathway trip (rule 4.4.3 '
                  f'allows it, but it costs time).')
        else:
            print(f'  These count as R1\'s own KFS pickups (R1 holds {R1_CAPACITY} '
                  f'anyway), so clearing them is "free".')
            free_slots = R1_CAPACITY - len(ordered)
            leftover = [b for b in sorted(r1) if b not in ordered]
            if free_slots > 0 and leftover:
                print(f'  R1 has {free_slots} free pickup slot(s) left: it may '
                      f'grab any of {leftover} for extra Arena scoring.')

    print(f"\nR2 collects: {route['collected']}   Exit: block {route['exit_block']}")
    pos_disp = ['Pathway' if p == START else str(p) for p in route['positions']]
    print(f"R2 foot-path: {' -> '.join(pos_disp)}  (+ descend off {route['exit_block']})")

    print('\nR2 action sequence:')
    for i, (a, c, _meta) in enumerate(generate_actions(route), 1):
        print(f'  {i:>2}. {a:<20}{("   # " + c) if c else ""}')


def main():
    print('=' * 64)
    print('R2 Meihua Forest Full Planner (with R1 coordination)')
    print('=' * 64)
    print('Enter the full Forest state. Entrance = block 2; exits = 10 or 12.')
    print()
    while True:
        r1 = parse_blocks('R1 KFS blocks (3, boundary only, e.g. 1 3 10): ', 3, set(), boundary_only=True)
        r2 = parse_blocks('R2 KFS blocks (4, e.g. 5 6 8 11): ', 4, set(r1))
        fake = parse_single('Fake KFS block (1, not 1/2/3): ', set(r1) | set(r2))
        ndp = input('Forbid downward picks (no_downward_pick)? (y/N): ').strip().lower() == 'y'
        report(r1, r2, fake, no_downward_pick=ndp)
        print()
        if input('Plan another? (y/n): ').strip().lower() != 'y':
            break
        print()


if __name__ == '__main__':
    main()
