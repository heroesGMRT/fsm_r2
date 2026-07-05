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
  moment it steps there.
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

Grid (display orientation, entrance row at bottom):
    10  11  12
    7   8   9
    4   5   6
    1   2   3

Heights (right-side team, A=200 B=400 C=600):
    1:B 2:A 3:B  4:C 5:B 6:A  7:B 8:C 9:B  10:A 11:B 12:A

FLAGGED RULE INTERPRETATIONS (please confirm -- they affect results):
[I1] Rule 4.4.14 "R2 must collect its first KFS from blocks 1,2,3 from the
     R2 Entrance Zone": implemented as "the FIRST R2 KFS R2 collects must be
     located on block 1, 2, or 3." Toggle: ENFORCE_FIRST_FROM_ENTRANCE.
[I2] Entrance-zone reach: while still on the Pathway (before climbing in),
     R2 may pick an R2 KFS off block 1, 2, or 3 (assuming it can move along
     the flat entrance zone in front of all three). Toggle: ENTRANCE_ZONE_REACH.
[I3] Entry is via block 2 only (sole tier-A entrance block); exits are 10 or
     12 only (sole tier-A exit blocks; 11 is tier B -> 2-tier drop disallowed).
[I4] Fake block treated as fully impassable; never stepped on.
"""

from collections import deque
from itertools import combinations

# ---------------------------------------------------------------------------
# Config toggles for flagged interpretations
# ---------------------------------------------------------------------------
ENFORCE_FIRST_FROM_ENTRANCE = True   # [I1]
ENTRANCE_ZONE_REACH = True           # [I2]
R2_CAPACITY = 2
R1_CAPACITY = 2
WANT_COLLECT = 2                     # R2 target number of R2 KFS to collect

# ---------------------------------------------------------------------------
# Field constants
# ---------------------------------------------------------------------------
GRID_COLS = 3
ALL_BLOCKS = list(range(1, 13))
ENTRANCE = 2                         # robot climbs in via block 2 (bottom-middle)
VALID_EXITS_RANKED = [12]            # robot always exits at block 12 (top-right)
START = 0                            # virtual Pathway / entrance-zone node
ENTRANCE_ROW = {1, 2, 3}            # bottom row, reachable from outside before climbing

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

# Physical terrain heights of each block (independent of scoring-tier heights).
# The robot can only step between ADJACENT physical levels (LOW<->MED, MED<->HIGH).
# LOW:  2 6 10 12
# MED:  1 3 5 7 9 11
# HIGH: 4 8
PHYSICAL_LEVEL = {
    0: 'GROUND',
    1: 'MED',  2: 'LOW',  3: 'MED',
    4: 'HIGH', 5: 'MED',  6: 'LOW',
    7: 'MED',  8: 'HIGH', 9: 'MED',
    10: 'LOW', 11: 'MED', 12: 'LOW',
}
PHYS_ORDER = {'GROUND': 0, 'LOW': 1, 'MED': 2, 'HIGH': 3}


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


def phys_adjacent(a, b):
    """True if the physical terrain levels of blocks a and b are adjacent
    (i.e. differ by at most one step: GROUND/LOW/MED/HIGH).
    START (virtual ground node) is always considered adjacent to ENTRANCE."""
    if a == START or b == START:
        return True
    return abs(PHYS_ORDER[PHYSICAL_LEVEL[a]] - PHYS_ORDER[PHYSICAL_LEVEL[b]]) <= 1


def move_neighbors(b):
    """Blocks R2 can CLIMB to from b (must also be passable, checked later).
    Only returns blocks whose physical terrain level is adjacent to b."""
    if b == START:
        return [ENTRANCE]          # robot enters via block 2 (bottom-middle)
    return [n for n in grid_neighbors(b) if phys_adjacent(b, n)]


def collect_neighbors(b):
    """Blocks R2 can REACH INTO to pick a KFS from position b (rule 4.4.15)."""
    if b == START:
        return list(ENTRANCE_ROW) if ENTRANCE_ZONE_REACH else [ENTRANCE]
    return grid_neighbors(b)


def direction_of(a, b):
    # From START (outside bottom row) the robot faces NORTH to look at block 2.
    if a == START:
        return 'NORTH'
    ra, ca = divmod(a - 1, GRID_COLS)
    rb, cb = divmod(b - 1, GRID_COLS)
    dr, dc = rb - ra, cb - ca
    return {(1, 0): 'NORTH', (-1, 0): 'SOUTH',
            (0, 1): 'EAST', (0, -1): 'WEST'}[(dr, dc)]


def rotation_needed(cur, tgt):
    return ((COMPASS.index(tgt) - COMPASS.index(cur)) % 4) * 90


# ---------------------------------------------------------------------------
# Core R2 search for a FIXED set of cleared R1 blocks
# ---------------------------------------------------------------------------
def r2_search(r1_blocks, r2_blocks, fake_block, cleared_r1, want=WANT_COLLECT):
    """
    Find shortest legal R2 plan given which R1 blocks are cleared.
    Returns dict(path, collected, exit_block, picks) or None.

    State = (position, frozenset(collected_blocks), first_done_flag).
    A block is foot-passable iff:
        - it's empty (no scroll), OR
        - it's an R2 KFS block already in `collected` (emptied on pickup), OR
        - it's an R1 block in `cleared_r1`.
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

    best = None  # (steps, ramp_rank, path, collected_order, exit_block)

    # Rule 4.4.14 only binds when at least one R2 KFS actually sits on the
    # entrance row (blocks 1/2/3). If none do, R2 is free to collect its
    # first KFS anywhere (per Wafdan's clarification).
    has_entrance_r2 = bool(r2_set & ENTRANCE_ROW)
    enforce_first = ENFORCE_FIRST_FROM_ENTRANCE and has_entrance_r2

    for exit_block in VALID_EXITS_RANKED:
        if not passable(exit_block, frozenset(r2_set)):
            # exit permanently blocked (Fake/R1-uncleared on it); collected
            # can't help unless exit itself is an R2 KFS we collect -> handled
            # by the search below, so don't prematurely skip R2-KFS exits.
            if exit_block not in r2_set:
                continue

        start_state = (START, frozenset(), False)
        # path stores list of (position, picks_made_at_this_position)
        visited = {start_state}
        queue = deque([(start_state, [(START, [])])])
        found = None

        while queue:
            (pos, collected, first_done), trail = queue.popleft()

            if pos == exit_block and len(collected) == want:
                found = trail
                break

            # --- Option 1: pick an adjacent R2 KFS (if capacity remains) ---
            if len(collected) < want:
                for tgt in collect_neighbors(pos):
                    if tgt in r2_set and tgt not in collected:
                        # Row-1 R2 KFS (blocks 1/2/3) must be picked from the
                        # Pathway (START), never by reaching back from an
                        # interior block after climbing in.
                        if tgt in ENTRANCE_ROW and pos != START:
                            continue
                        # 4.4.14 (only when an entrance-row R2 KFS exists):
                        # the FIRST collected KFS must be on the entrance row.
                        if (enforce_first and not first_done
                                and tgt not in ENTRANCE_ROW):
                            continue
                        new_collected = collected | {tgt}
                        new_state = (pos, new_collected, True)
                        if new_state not in visited:
                            visited.add(new_state)
                            # record pick at current position
                            new_trail = trail[:-1] + [(pos, trail[-1][1] + [tgt])]
                            queue.append((new_state, new_trail))

            # --- Option 2: climb to an adjacent passable block ---
            # 4.4.14 guard: when an entrance-row R2 KFS exists, R2 may not
            # leave the entrance zone (START) before making its first
            # (entrance-row) collection. When none exists, R2 climbs freely.
            block_climb_from_start = (enforce_first and not first_done
                                       and pos == START)
            if not block_climb_from_start:
                for nxt in move_neighbors(pos):
                    if not passable(nxt, collected):
                        continue
                    new_state = (nxt, collected, first_done)
                    if new_state not in visited:
                        visited.add(new_state)
                        queue.append((new_state, trail + [(nxt, [])]))

        if found is None:
            continue

        steps = sum(1 for (p, _) in found if p != START) - 0  # block hops
        # simpler: number of climbs = len of positions excluding START minus
        # transitions; use path length
        positions = [p for (p, _) in found]
        steps = len(positions) - 1
        ramp_rank = VALID_EXITS_RANKED.index(exit_block)
        key = (steps, ramp_rank)
        if best is None or key < (best[0], best[1]):
            best = (steps, ramp_rank, found, exit_block)

    if best is None:
        return None

    steps, ramp_rank, trail, exit_block = best
    positions = [p for (p, _) in trail]
    picks_at = {p: picks for (p, picks) in trail}
    collected_order = [b for (_, picks) in trail for b in picks]
    return {
        'positions': positions,         # includes START at front
        'trail': trail,                  # list of (pos, [picks at pos])
        'collected': collected_order,
        'exit_block': exit_block,
        'steps': steps,
    }


# ---------------------------------------------------------------------------
# Top-level planner
# ---------------------------------------------------------------------------
def plan(r1_blocks, r2_blocks, fake_block, want=WANT_COLLECT):
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
                                  cleared_r1=set(clear_combo), want=target)
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
    trail = route['trail']
    exit_block = route['exit_block']
    actions = []
    facing = 'NORTH'

    def emit(a, c=None):
        actions.append((a, c))

    def face(direction, desc):
        nonlocal facing
        rot = rotation_needed(facing, direction)
        if rot:
            emit(f'ROTATE_{rot}', f'turn to face {desc}')
            facing = direction

    def do_picks(pos, picks):
        # interior picks: rotate to face the adjacent block, then pick.
        for tgt in picks:
            face(direction_of(pos, tgt), f'block {tgt}')
            emit('VISUAL_SERVO_BLOCK', f'align pickup to block {tgt}')
            ref_tier = TIER_LEVEL['GROUND'] if pos == START else TIER_LEVEL[HEIGHTS[pos]]
            if TIER_LEVEL[HEIGHTS[tgt]] > ref_tier:
                emit('PICK_BLOCK_UP', f'collect R2 KFS at block {tgt} (one tier above)')
            else:
                emit('PICK_BLOCK_DOWN', f'collect R2 KFS at block {tgt} (one tier below)')

    # Entrance-zone column of each bottom-row block (0=left/block1, 1=block2, 2=right/block3)
    # Robot starts outside the bottom-middle, in front of block 2 (column 1).
    ENTRANCE_COL = {1: 0, 2: 1, 3: 2}

    def do_start_picks(picks):
        """R2 begins on the Pathway in front of block 2 (column 1), facing
        NORTH. To pick a bottom-row KFS on block 1 or 3 it strafes LEFT/RIGHT
        along the entrance zone, picks, then strafes back to column 1 to
        climb in via block 2."""
        lateral = 1  # column R2 starts at (in front of block 2)

        def strafe_to(col, why):
            nonlocal lateral
            while lateral < col:
                emit('STRAFE_RIGHT', why)
                lateral += 1
            while lateral > col:
                emit('STRAFE_LEFT', why)
                lateral -= 1

        # sort picks by column so R2 sweeps rather than zig-zags
        for tgt in sorted(picks, key=lambda b: ENTRANCE_COL[b]):
            strafe_to(ENTRANCE_COL[tgt], f'strafe along entrance zone to face block {tgt}')
            emit('VISUAL_SERVO_BLOCK', f'align pickup to block {tgt}')
            emit('PICK_BLOCK_UP', f'collect bottom-row R2 KFS at block {tgt} from the Pathway')

        if picks:
            strafe_to(1, 'strafe back to face entrance block 2')

    # ── Entry sequence ────────────────────────────────────────────────────────
    # Robot approaches from outside the bottom-middle, then lifts itself up
    # onto the jungle blocks via the lift-cross sequence.
    emit('FORWARD_INIT', 'approach from 1m outside block 2, bottom-middle entrance')
    emit('LIFT_CROSS_SEQUENCE', 'lift both wheels → forward → front down → forward → back down → forward: climb up onto jungle at block 2')

    # bottom-row picks from the Pathway before climbing in
    do_start_picks(trail[0][1])

    # ── Traverse the forest ───────────────────────────────────────────────────
    for i in range(len(trail) - 1):
        a = trail[i][0]
        b, picks_b = trail[i + 1]
        face(direction_of(a, b), f'block {b}')
        from_label = 'Pathway (ground)' if a == START else f'block {a} ({HEIGHTS[a]}, {PHYSICAL_LEVEL[a]})'
        to_label   = f'block {b} ({HEIGHTS[b]}, {PHYSICAL_LEVEL[b]}, {TIER_MM[HEIGHTS[b]]}mm)'
        if TIER_LEVEL[HEIGHTS[b]] > (TIER_LEVEL['GROUND'] if a == START else TIER_LEVEL[HEIGHTS[a]]):
            emit('CLIMB_UP',   f'{from_label} -> {to_label}')
        else:
            emit('CLIMB_DOWN', f'{from_label} -> {to_label}')
        do_picks(b, picks_b)

    # ── Exit sequence (block 12, top-right) ───────────────────────────────────
    # Robot descends from block 12 back to ground level using a 6-step sequence.
    emit('CLIMB_DOWN',
         f'block {exit_block} ({HEIGHTS[exit_block]}, {PHYSICAL_LEVEL[exit_block]}) -> Pathway (ground), EXIT Forest')
    emit('DESCENT_FORWARD',     'move forward off block 12')
    emit('EXTEND_FRONT_WHEEL',  'lower front wheel to ground')
    emit('DESCENT_FORWARD',     'move forward: front wheel on ground, rear still elevated')
    emit('EXTEND_BACK_WHEEL',   'lower back wheel to ground')
    emit('DESCENT_FORWARD',     'move forward: both wheels on ground')
    emit('RETRACT_BOTH_WHEELS', 'retract both wheels to normal driving position')
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


def report(r1, r2, fake):
    result = plan(r1, r2, fake)
    print()
    print(f'R1 KFS : {sorted(r1)}')
    print(f'R2 KFS : {sorted(r2)}')
    print(f'Fake   : {fake}')
    empties = [b for b in ALL_BLOCKS if b not in set(r1) | set(r2) | {fake}]
    print(f'Empty  : {empties}')

    route = result['route']
    clear_set = result['clear_set']

    if result['status'] == 'impossible':
        print_grid(r1, r2, fake)
        print('RESULT: NO LEGAL PATH, even if R1 clears all of its KFS.')
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
    for i, (a, c) in enumerate(generate_actions(route), 1):
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
        report(r1, r2, fake)
        print()
        if input('Plan another? (y/n): ').strip().lower() != 'y':
            break
        print()


if __name__ == '__main__':
    main()