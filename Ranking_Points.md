# BIOBUZZ Ranking Points

This document covers every **Ranking Point (RP)** awarded in a
**Qualification Match** under Competition Manual V1, Table 10-2. RPs are
awarded to each eligible team based on its two-robot alliance's performance.
They are not awarded in Playoffs.

## At a glance

| RP | Award | Manual V1 requirement at non-premier events other than Regional Championships | What the alliance needs |
| --- | --- | --- | --- |
| SWARM RP | 1 RP | At least 16 combined LEAVE and PARK points | Plan AUTO departure and Loading-Zone parking that total at least 16 points. |
| POLLINATOR 1 RP | 1 RP | At least 4 HIVE TIPS | Repeatedly collect and launch scoring elements into the alliance HIVE's upward-facing Cell. |
| POLLINATOR 2 RP | 1 RP | At least 7 HIVE TIPS | The same HIVE-tipping system, but with substantially higher throughput and consistency. |
| WIN RP | 3 RP | Finish with more Match points than the opposing alliance | Score more total points than the opponent after scoring and any foul adjustments. |
| TIE RP | 1 RP | Finish with the same Match points as the opposing alliance | Match the opponent's final score. This is mutually exclusive with the WIN RP. |

The SWARM, POLLINATOR 1, and POLLINATOR 2 RPs are independent:
an alliance can earn all three in one Qualification Match. A winning alliance
that earns all three earns **6 RPs**; a tied alliance that earns all three earns
**4 RPs**.

## How qualification ranking works

The Event Management System awards each RP **after each Qualification Match**.
For every eligible team, the system calculates:

```text
Ranking Score (RS) = total Qualification-Match RPs / non-surrogate Qualification Matches
```

Higher RS ranks first. If teams have the same RS, the tiebreakers, in order,
are:

1. Average alliance Match points **excluding** Minor and Major Foul points.
2. Average number of HIVE TIPS.
3. Average AUTO points.
4. Random event-management-system sort.

A Surrogate Match is excluded from *every* ranking calculation. A
disqualified team earns zero Match points and zero RPs for that Match, and
that Match contributes zero to all of its ranking tiebreakers. In
Qualifications, the partner is not disqualified merely because its alliance
partner was disqualified.

## Qualification Match timeline

| Period | Duration | Driver control | RP-relevant work |
| --- | ---: | --- | --- |
| AUTO | 30 seconds | No; only the selected pre-programmed OpMode operates | Earn LEAVE, AUTO PARK, AUTO HIVE Tip points, and begin the Tip count. |
| Transition | 8 seconds | No scoring movement allowed | Robot and every powered mechanism must remain motionless. |
| TELEOP | 2 minutes | Yes | Finish HIVE Tips, score FLOWERS/GARDEN, and TELEOP PARK. |
| After 0:00 | Until scoring is complete | No powered movement | Let the robot and elements settle; referees make final end-of-Match assessments. |

Every robot starts the Match contacting four pre-loaded POLLEN. A launcher
robot can use these pieces during AUTO, but its autonomous program must run
without driver input and avoid disrupting the opponent on its side of the
FIELD.

## All Match-point sources that affect WIN, TIE, and tiebreakers

| Action | AUTO Match points | TELEOP Match points | Exact scoring condition |
| --- | ---: | ---: | --- |
| LEAVE | 3 | - | At AUTO end, robot no longer contacts the perimeter wall. |
| PARK | 5 | 5 | At the applicable assessment time, robot is at least partially in its alliance Loading Zone. |
| HIVE TIP | 20 | 20 | HIVE changes stable state after a legal launch into its upward-facing Cell. |
| POLLEN/NECTAR in upward-facing CELL | - | 2 each | Element remains in the upward-facing Cell after all elements and robots settle at Match end. |
| Bottom NECTAR bonus | - | 5 | Alliance has its color's bottom-most scoring NECTAR in a FLOWER. |
| Element in owned FLOWER | - | 2 each | POLLEN or NECTAR is at least partly in the FLOWER scoring volume, and the alliance owns that FLOWER. |
| Element in GARDEN | - | 1 each | POLLEN or NECTAR is at least partly in the alliance-color GARDEN. |

**Important attribution rules:**

- The alliance with its color's **top-most** scoring NECTAR owns a FLOWER and
  receives every point for all elements scoring in that FLOWER, including
  elements placed by the opponent.
- A GARDEN scores for the alliance matching the GARDEN color, regardless of
  who placed the element. GARDENS are not protected; either alliance may remove
  elements during the Match.
- FLOWER scoring cannot begin until the final 60 seconds. Enter pieces only
  through the top; remove only POLLEN through the bottom.
- A HIVE Tip may be scored through the end of the Match, but only when the
  elements and robots have stopped moving is the final end-state scoring
  determined.

## 1. SWARM RP - 1 RP

**Award condition:** The alliance's combined **LEAVE + PARK** Match points
must be at least **16**. For ordinary non-premier events other than Regional
Championships, the threshold is 16 points. The Regional Championship and FIRST
Championship thresholds are **TBA** in Manual V1; a Premier Event may set its
own threshold.

### Exact scoring interpretation

- Each robot earns **3 AUTO points** for a LEAVE when, at the end of AUTO, it
  is no longer contacting the perimeter wall.
- A robot earns **5 AUTO points** for PARK when it is at least partially inside
  its alliance-specific Loading Zone at the end of AUTO, and **5 TELEOP points**
  when it meets the same condition at the end of the Match. Those assessments
  occur at separate times.
- A simple 16-point route is both alliance robots earning a LEAVE (2 x 3 = 6)
  and both robots TELEOP PARKing (2 x 5 = 10). A higher-margin plan can also
  include AUTO PARKs, provided the robot is in the Loading Zone when AUTO ends.

### Robot and alliance requirements

1. **Reliable autonomous drive:** The robot must run a 30-second autonomous
   routine without driver input, clear the starting perimeter wall, and stop in
   a known safe location. Dead-reckoning can be sufficient for LEAVE, but
   encoders/odometry, IMU heading control, and/or AprilTag localization reduce
   failure from drift and collisions.
2. **Partner-compatible AUTO routes:** Both robots must clear their respective
   walls without blocking or contacting each other. Coordinate starting
   positions, first movement direction, and any shared field-space crossings
   before the Match.
3. **Endgame parking capability:** Build and program a short, repeatable route
   to the alliance Loading Zone. The chassis and any extended mechanisms must
   fit at least partially inside the zone without being pushed back out before
   scoring is assessed.
4. **Positive driver feedback:** A driver should be able to identify the
   Loading Zone boundary and confidently stop with part of the robot over it.
   Camera localization, odometry, field-alignment features, or a simple
   driver-visible alignment reference are useful. Do not rely on a borderline
   position; make the robot visibly inside the zone for the human scorekeepers.
5. **Mechanism stow/reliability:** No scoring mechanism is inherently required
   for this RP, but mechanisms must stow so they do not snag the wall, prevent
   departure, obstruct the partner, or make final parking unreliable.

### Example SWARM plans

| Plan | AUTO result | TELEOP-end result | LEAVE + PARK total |
| --- | --- | --- | ---: |
| Minimum clear route | Both robots LEAVE (6); neither AUTO PARKs | Both robots PARK (10) | 16 |
| Higher-margin route | Both robots LEAVE (6); one robot AUTO PARKs (5) | Both robots PARK (10) | 21 |
| Park-focused route | One robot LEAVEs (3); both AUTO PARK (10) | Both robots PARK (10) | 23 |

The first plan is usually the simplest: each robot moves cleanly away from the
wall in AUTO, performs its main task, then returns to be visibly inside its
Loading Zone before the Match ends.

## 2. POLLINATOR 1 RP - 1 RP

**Award condition:** The alliance earns this RP when it records **4 or more
HIVE TIPS**. The non-premier, non-Regional-Championship threshold in Manual V1
is 4 Tips. Regional Championship and FIRST Championship thresholds are TBA;
Premier Events may choose their own threshold.

### What counts as a HIVE TIP

The alliance HIVE must move from one of its two stable states to the other:
the previously downward-facing Cell becomes upward-facing, and the previously
non-contacting HIVE damper begins contacting the frame. A Tip is only legal
when caused by **launching POLLEN or NECTAR into the upward-facing Cell**.
Touching, pushing, ramming, blocking, or otherwise manipulating the HIVE is
not a substitute for a launch and can draw a Major Foul and Yellow Card.

### Robot and alliance requirements

1. **Legal launch mechanism:** Use a repeatable launcher that releases POLLEN
   and NECTAR with enough height, range, and energy to enter the *currently
   upward-facing* Cell. A flywheel, wheel shooter, hooded roller launcher, or
   other controlled launcher is appropriate; the mechanism must not be designed
   to strike the HIVE or Cell exterior.
2. **Collection and indexing:** The robot needs an intake, conveyor/indexer,
   or equivalent transfer path that acquires variable-size game pieces and
   presents them consistently to the launcher. POLLEN and NECTAR are not
   perfectly spherical, so the intake, centering, and shooter throat need
   tolerance for dimensional variation.
3. **Four-piece maximum:** The robot may not simultaneously control more than
   four scoring elements. Design the magazine/indexer with a physical or
   software limit of four; include guards so loose pieces cannot accumulate on
   top of or inside the robot as unintended extra possession.
4. **Targeting after every Tip:** The HIVE is bi-stable: after each successful
   Tip, its other Cell becomes the upward-facing target. The launcher must
   either cover both target positions or the chassis must quickly reposition and
   re-aim after each Tip. The two Cells are approximately 18.8 in. apart, and
   the Cell opening is approximately 20 in. wide by 14 in. tall by 12 in. deep.
5. **Tip confirmation and shot timing:** Do not keep firing at the
   downward-facing Cell while the HIVE is moving. Pause long enough for the
   HIVE to settle and for the Tip to be obvious, then switch to the new
   upward-facing Cell. Vision, a shot-count/state model, or driver observation
   should determine when to change target.
6. **Sustained cycle speed:** Four Tips require more than a one-shot autonomous
   launcher. The alliance needs rapid collect-load-aim-launch cycles throughout
   AUTO and TELEOP, plus a division of labor with its partner so both robots do
   not interfere around the HIVE.

Each completed Tip also unlocks one of the five alliance-color NECTAR initially
held in the ALLIANCE AREA for entry by the HUMAN PLAYER. At 60 seconds
remaining, all still-locked NECTAR becomes available. This makes early, reliable
Tips valuable: they increase the Tip count and release more potential
endgame/FLOWER scoring pieces.

## 3. POLLINATOR 2 RP - 1 RP

**Award condition:** The alliance earns this RP when it records **7 or more
HIVE TIPS**. Its event-threshold status is the same as POLLINATOR 1: 7 Tips at
ordinary non-premier events, TBA at Regional Championships and FIRST
Championship, and event-defined at Premier Events.

### Additional requirement beyond POLLINATOR 1

This RP uses the identical legal scoring action as POLLINATOR 1, so every
launcher, intake, capacity, target-switching, and HIVE-contact restriction
above applies. It is not a different type of Tip and is awarded in addition to,
not instead of, POLLINATOR 1.

### Robot and alliance requirements

1. **High-throughput launcher:** The launcher must be repeatable across at
   least seven HIVE state changes, not merely capable of one or two long-range
   shots. Prioritize consistent exit speed, retention while driving, rapid
   reloading, and jam recovery.
2. **Fast acquisition from legal sources:** Build an intake that can collect
   POLLEN from the floor and from FLOWERS, and safely process alliance NECTAR
   once it is available. Do not control opponent NECTAR.
3. **Efficient field navigation:** Seven Tips require short travel paths,
   rapid alignment, and avoidance of congestion near the center HIVE. A
   holonomic drive, localization, and planned partner lanes are advantageous
   because a slow turn-and-aim cycle will consume too much of the 2:30 Match.
4. **Autonomous contribution:** AUTO is an opportunity to start the Tip count
   before drivers take over. An autonomous routine may collect the four
   pre-loaded POLLEN, navigate, aim, and launch without driver input. It must
   respect the opponent's AUTO priority and avoid disrupting the opposing
   alliance's autonomous routine.
5. **Alliance coordination:** Decide which robot gathers, launches, clears
   dropped pieces, and parks at each stage. Both robots can contribute, but
   uncontrolled simultaneous traffic at the HIVE increases missed shots,
   collisions, and the chance of disrupting an otherwise valid Tip.

Seven Tips does not require seven elements in a single robot or a special
seven-piece storage mechanism. The legal model is repeated cycles of up to four
controlled elements: acquire, index, launch at the upward-facing Cell, confirm
the Tip, change target, and repeat.

## 4. WIN RP - 3 RP

**Award condition:** Earned by every eligible team on the alliance that
finishes a Qualification Match with **more total Match points** than its
opponent.

### Robot and alliance requirements

There is no single mandatory mechanism for a WIN RP. The robot instead needs a
scoring package and strategy that out-scores the opponent while protecting
earned points:

1. **Choose reliable scoring priorities:** HIVE Tips are worth 20 Match points
   in either AUTO or TELEOP, with 2 points for each POLLEN or NECTAR remaining
   in the upward-facing Cell at Match end. FLOWER scoring yields 2 points for
   every POLLEN/NECTAR in an owned FLOWER plus a 5-point bottom-NECTAR bonus;
   GARDEN scoring is 1 point per element. Design around the actions the robot
   can repeat accurately under defense and alliance traffic.
2. **Use a two-robot strategy:** Coordinate a complementary scorer, such as a
   high-rate HIVE launcher paired with a collector/Flower scorer, or two
   launch-capable robots with defined field lanes. The alliance score, not an
   individual robot's score, decides this RP.
3. **Score FLOWERS legally and on time:** A Flower mechanism must place pieces
   into the *top* of a Flower; it may remove only POLLEN from the bottom.
   Robots may not enter NECTAR into the Flower scoring volume until the final
   60 seconds of the Match.
4. **Avoid avoidable opponent points:** Do not commit Minor Fouls (5 points
   credited to the opponent) or Major Fouls (20 points). Avoid damaging field
   elements, intentionally ejecting pieces, hoarding pieces from the opponent,
   or entering the opponent's side during AUTO in a way that disrupts them.
5. **Finish scoring cleanly:** Stop all powered robot/mechanism movement at
   the end of TELEOP. Ensure scoring pieces and the robot come to rest in an
   unambiguous scoring state; a late movement can change scoring or create a
   penalty.

## 5. TIE RP - 1 RP

**Award condition:** Earned by every eligible team on an alliance that finishes
a Qualification Match with **exactly the same total Match points** as the
opposing alliance.

### Robot and alliance requirements

This is not a separate task a robot can execute on demand. It is the outcome of
the final score, so it requires the same legal, reliable scoring capability as
a WIN RP plus score awareness:

1. **Live-score awareness:** Drivers and coaches need to track the official
   Match score and their likely unscored/end-of-Match points. Use that
   information to decide whether one more safe scoring cycle is likely to win,
   or whether preserving the tie is the lower-risk outcome.
2. **Fine score control:** A robot should be able to stop collecting or
   launching promptly, retain or safely place a piece, and park without an
   accidental late score. This matters because HIVE, Flower, Garden, and PARK
   results can be assessed after elements and robots come to rest.
3. **Rule compliance remains essential:** A tie can be lost to a foul.
   Robust driver controls, predictable mechanisms, protected wiring, and
   non-damaging contact surfaces are design requirements for preserving a
   close result.

## What an RP-focused robot should look like

The following is a practical design target, not an additional set of game
rules. It is the smallest broadly capable package that can pursue every RP
rather than depending entirely on its alliance partner.

| Subsystem | Recommended capability | Why it matters to RPs |
| --- | --- | --- |
| Drivetrain | Fast, controllable holonomic drive with reliable braking and a stowed footprint that can enter the Loading Zone | LEAVE, AUTO/TELEOP PARK, HIVE alignment, and avoiding alliance congestion. |
| Localization | Wheel odometry and IMU at minimum; AprilTag/HIVE-target vision if reliable | Repeat AUTO departure, identify which Cell is upward, hold a shooting pose, and park unambiguously. |
| Floor/Flower intake | Wide, compliant intake that collects POLLEN from the floor and may remove POLLEN from a Flower's bottom | Sustains the repeated collection needed for 4 and 7 HIVE Tips. |
| Four-piece storage/indexer | Centring path, piece sensing, controlled feed, and a hard maximum of four controlled elements | Feeds the launcher efficiently while satisfying G407. |
| Launcher | Consistent launch speed and angle, rapid single-piece feed, safe miss behavior, and quick re-aim between Cell positions | The essential mechanism for both POLLINATOR RPs and high-value Match scoring. |
| Tip-state tracking | Vision, driver indication, or a reliable internal state that changes only after a confirmed Tip | Prevents firing at the now-downward Cell or incorrectly counting Tips. |
| Flower placer | Optional top-entry placement geometry, preferably able to place NECTAR deliberately during the final minute | Adds Match points and Flower ownership to pursue WIN RP. |
| Driver interface | Launcher-ready indication, four-piece/capacity indication, match timer, Tip count, and a parking cue | Turns a capable mechanism into repeatable scoring under time pressure. |

### Recommended operating roles for a two-robot alliance

**Launcher robot:** collects, indexes, aligns to the upward-facing Cell, fires
a controlled burst, waits for the HIVE to finish tipping, then changes targets.
It can use the four AUTO pre-loads to begin the Tip count.

**Support/finisher robot:** collects loose POLLEN, clears pieces only after
they have contacted the floor following a Tip, adds launches or feeds the
launcher robot's lane, and shifts to legal FLOWER scoring in the final minute.
Both robots reserve enough time and a clear lane to PARK.

Roles are not restrictions. Two launch-capable robots can be stronger, but they
must designate who owns the HIVE approach at a given time. Simultaneously
driving or launching through the same space increases collision, missed-shot,
and HIVE-interference risk.

## Match plans that earn RPs

### Plan A: Reliable four-RP target

1. In AUTO, both robots LEAVE; the launcher robot shoots pre-loaded POLLEN
   only when the target Cell is upward and then either AUTO PARKs or clears a
   partner lane.
2. In TELEOP, the alliance cycles intake -> four-piece-or-fewer storage ->
   launcher until it reaches four confirmed Tips.
3. The support robot starts FLOWER work only after the final minute begins and
   places NECTAR through the top to gain or retain ownership.
4. Before time expires, both robots TELEOP PARK. This earns the 16-point
   SWARM condition when both LEAVEs and both TELEOP PARKs are scored.
5. Continue or protect scoring so the alliance finishes ahead. The result is
   SWARM + POLLINATOR 1 + WIN = **5 RPs**.

### Plan B: Maximum ordinary-event RP target

1. Execute the SWARM actions above, including both robots' reliable endgame
   PARK.
2. Reach and confirm **seven** legal Tips, which earns both POLLINATOR RPs:
   four Tips crosses POLLINATOR 1, and seven crosses POLLINATOR 2.
3. Use surplus time for the final-minute FLOWER ownership/element points or
   other low-risk scoring, then protect the lead.
4. Win the Match: SWARM (1) + POLLINATOR 1 (1) + POLLINATOR 2 (1) + WIN (3)
   = **6 RPs**, the maximum from the Manual V1 table.

### Plan C: When the score is close

At 60 seconds, assign the coach to track the official score, likely HIVE/Flower
end-state scores, and both alliances' parking status. With a lead, favor
unambiguous PARK and low-risk element placement over a risky extra shot. When
tied, a final legal score can change the TIE RP into the more valuable WIN RP;
do not deliberately rely on an ambiguous final piece or borderline PARK.

## RP-protection checklist

- **Never control more than four** POLLEN/NECTAR at once; engineer the robot so
  accidental fifth-piece capture is difficult.
- **Never control opponent NECTAR.**
- **Never catch/deflect pieces dropping from a tipped HIVE** before they touch
  something other than the robot. Do not wait below the HIVE with an open
  collector.
- **Never use the robot or a controlled piece to move, impede, or tip a HIVE.**
  Launch only into the currently upward-facing Cell; avoid deliberate exterior
  Cell hits and HIVE-frame contact.
- **Do not launch or place NECTAR into a FLOWER before the last 60 seconds.**
- **Keep scoring elements in bounds** and do not damage the FIELD, HIVE,
  FLOWERS, or game pieces.
- **Stop powered movement** during the AUTO-to-TELEOP transition and after
  TELEOP ends.
- A Minor Foul awards the opponent 5 Match points; a Major Foul awards 20.
  Either can turn a WIN into a TIE or loss, and a disqualification removes that
  team's RPs entirely.

## Ranking and eligibility notes

- A team's **Ranking Score (RS)** is its average RPs across Qualification
  Matches, excluding surrogate Matches. Teams are ranked first by RS.
- A disqualified team receives **0 Match points and 0 RPs** for that
  Qualification Match. Its alliance partner is not disqualified solely because
  of that team's Qualification-Match disqualification.
- A disabled or absent robot can still receive Qualification Match points if it
  passed inspection and at least one student DRIVE TEAM member is in the
  ALLIANCE AREA. That does not create an RP by itself; the alliance must still
  meet the relevant award condition.
- After RS, the ranking tiebreakers are average alliance Match points excluding
  fouls, average HIVE Tips, average AUTO points, then the event-management
  system's random sort.
- RPs are a Qualification-only ranking measure. Playoff advancement is based on
  winning and losing Matches, not RPs.

## Manual references

- Competition Manual V1: Sections 10.4-10.5.5; Tables 10-2 and 10-3.
- Scoring/legality details: Sections 10.5.1-10.5.4 and rules G401-G418,
  especially G407 (four-element control limit), G408 (opponent NECTAR), G409
  (no catching from a tipped HIVE), G410 (Flower timing), G417 (HIVE), and
  G418 (Flowers).
- Qualification ranking: Section 13.6.3 and Table 13-1.
