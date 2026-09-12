# BIOBUZZ Autonomous Plan

## Objective

Score as many 20-point HIVE TIPS as possible while guaranteeing this robot's
8 AUTO points:

- 3 points for LEAVE: no longer touch the perimeter wall at any point in AUTO.
- 5 points for AUTO PARK: end at least partially inside the alliance LOADING
  ZONE.

Do not score NECTAR in a FLOWER: it is illegal until 1:00 remains in the
MATCH. HIVE TIPS are the only high-value autonomous objective.

## Pre-match setup

1. Start legally on our half, against the perimeter, outside the LOADING ZONE
   and FLOWER scoring volumes, with exactly four Pollen.
2. Aim the turret at our HIVE's currently upward-facing CELL using the
   AprilTag cluster. Start close enough to shoot reliably, but not where a
   tip can drop elements onto the robot.
3. Coordinate with the alliance partner before the match:
   - Assign one robot to call/detect the HIVE state.
   - Divide sources: this robot starts with the GARDEN; the partner starts
     with the nearer FLOWER, or reverse this assignment.
   - Never cross to the opponent half unless the partner has verified it is
     empty and the path cannot disrupt the opponent.

## 0:00-0:05 - Leave and score preloads

1. Drive away from the starting wall immediately to earn LEAVE.
2. Keep the turret locked on the upward-facing CELL.
3. Shoot the four preloads quickly, but stop immediately when Hive-state
   vision indicates the HIVE has begun to tip.
4. Hold fire and stay clear until the HIVE has fully reached its new stable
   position. Do not try to catch the released elements; they must first touch
   something other than the robot.
5. Reacquire the AprilTags and aim at the new upward-facing CELL.

## 0:05-0:13 - First refill: GARDEN

1. Move to the alliance GARDEN and intake up to four Pollen. The GARDEN
   begins with four Pollen, so it is the fastest predictable refill.
2. Use vision only to correct for shifted/missing Pollen and to stop at four;
   never deliberately control a fifth element.
3. While returning toward a safe shooting pose, let the turret track the
   upward-facing CELL.
4. Shoot the batch. Again, pause as soon as a tip begins, wait for it to
   settle, and then re-target the newly upward-facing CELL.

## 0:13-0:22 - Second refill: FLOWER

1. Drive to the assigned alliance-side FLOWER.
2. Intake Pollen only through its bottom retrieval opening. Do not attempt to
   remove NECTAR or use the side/top of the FLOWER for retrieval.
3. Use vision to find loose Pollen first; if the FLOWER path is blocked,
   select the closest legal, alliance-side Pollen source instead.
4. Stop intaking at four, return to a clear shooting pose, and shoot into the
   current upward-facing CELL.
5. If the HIVE tips, wait for the released pieces to hit the floor, then
   re-localize from its new AprilTags before continuing.

## 0:22-0:26 - Conditional last scoring cycle

1. If holding Pollen and a clear shot exists, shoot only until either the
   HIVE starts to tip or 0:26 is reached.
2. If empty, use vision to select only a nearby, loose Pollen that can be
   collected and shot before 0:26. Otherwise, skip the cycle.
3. Do not drive under the HIVE to wait for or collect a spill. Do not cross
   into the opponent half for this cycle.

## 0:26-0:30 - Park

1. Stop all collection and shooting actions.
2. Drive to our LOADING ZONE, ending at least partially inside it before AUTO
   expires.
3. Make all mechanisms motionless by 0:30; no powered movement is allowed in
   the 8-second AUTO-to-TELEOP transition.

## Runtime decision rules

| Condition | Robot action |
| --- | --- |
| Upward CELL is visible and stable; robot holds Pollen | Shoot at our HIVE. |
| Tip begins or HIVE state is uncertain | Stop shooting, move clear, wait for the stable state, then reacquire tags. |
| Holds fewer than four Pollen and GARDEN Pollen remain | Intake from our GARDEN. |
| GARDEN is empty or occupied | Intake Pollen through the bottom of our assigned FLOWER. |
| Both fixed sources are unavailable | Use vision to choose the closest loose Pollen on our alliance half. |
| A candidate path enters the opponent half | Reject it by default. Use it only when it is clearly empty and cannot affect the opponent's AUTO; their alliance has priority there. |
| More than 4 seconds remain before park deadline | Continue the current safe scoring cycle. |
| 4 seconds or fewer remain before AUTO ends | Abandon collection and AUTO PARK. |

## Non-negotiable boundaries

- Never control more than four scoring elements.
- Never control opponent NECTAR.
- Never intentionally eject an element from the FIELD.
- Only tip a HIVE by launching an element into its upward-facing CELL; never
  ram, contact, or impede the HIVE.
- Do not interfere with the opponent's autonomous. Crossing their half is
  legal but strategically risky and can become a Major Foul if disruptive.
- Do not use a FLOWER to score NECTAR during AUTO.
