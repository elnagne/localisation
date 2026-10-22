# CSC C85 Project 2: Robot Localization - Report Answers

Team members: 
ShenYi Wang 1010016536
Allan Zhao 1011100746
Ricky Su Shen 1007284163


These answers describe the current implementation in `EV3_Localization.c` and `EV3_Localization.h`. Numerical performance results below come from robot-free tests run on October 22, 2026. They are not measurements of physical robot performance. The optional feedback at the end is suggested wording for the team to review.

## Q1. How many probabilities are needed for an m by n map?

We keep **4mn probabilities**: one for each of the mn intersections and each of the four possible headings. Heading IDs are 0 = up, 1 = right, 2 = down and 3 = left. The implementation stores the distribution in `beliefs[400][4]`, supporting up to 400 intersections. The supplied Map1 has 3 by 5 intersections, so 60 entries are active.

## Q2. What should the initial probabilities be?

Initially every position-heading combination is equally likely, so each probability is **1/(4mn)**. For Map1, each is 1/60. The sum over all states is therefore one. `run_mission()` restores this uniform distribution before every localization attempt, including retries after losing the pose, and clears the colour-filter and alignment history.

## Q3. How do we handle sensor noise, why should this work, and how was it tested?

We combine calibration, repeated measurements and conservative handling of ambiguous readings.

1. **Calibrated colour classification.** The calibration routine collects 30 RGB samples per colour, spread over three distinct spots, for black, blue, green, yellow, red and white. Classification compares HSV features against these samples, using circular hue distance and saturation/brightness differences. Black/white shortcut thresholds are enabled only when the calibration data separates the relevant ranges. If the nearest class is too close to its runner-up, the reading becomes unknown.
2. **Repeated readings.** The general temporal filter requires three consecutive matches before changing its confirmed colour and expires an unsupported old result after three samples. Intersection scans use fresh local voting instead: the robot stops at each angular target and accepts a colour only after four agreeing readings out of at most five. This prevents the previous angular sample's colour from being carried into a different corner.
3. **Agreement across a corner sector.** A corner colour requires at least three agreeing angular samples and at least 70% of the sector's valid building-colour votes, with no tie. Blue, green and white can vote; white represents an absent building. Insufficient or conflicting evidence leaves the corner unknown.
4. **Event and communication checks.** Street events use four-of-five agreement, and a red boundary receives an additional stationary confirmation. The sensor readers validate reply framing and reject invalid data. Read or motion failures stop an operation rather than becoming landmark evidence.

Calibration adapts classification to the sensor and printed map. Repeated votes suppress isolated errors, stopped sampling reduces motion/communication latency effects, and unknown results avoid forcing weak evidence into a wrong colour. These methods cannot remove a systematic error, such as incorrectly labelled calibration samples or a misaligned scan.

Fresh robot-free tests passed for colour classification/filtering, EV3/NXT reader framing, sensor detection and calibration matching, gyro validation, and intersection sweeps. The sweep tests include missing roads, uncertain corners, return-heading checks and failures. These establish software behaviour; physical accuracy under different lighting and sensor placements still requires hardware trials.

## Q4. How do we define agreement and update probabilities?

For every candidate intersection and heading h, we compare the current scan's four robot-relative corners in clockwise order: TL, TR, BR, BL. The map stores NW, NE, SE, SW, so observed corner c is compared with `map_corners[(c+h)%4]`. This tests all four orientations of every intersection.

Each accepted corner contributes a likelihood factor:

- Correct colour: **0.8**.
- Either incorrect building colour: **0.1**.
- Unknown or unusable corner: **1**, meaning no evidence for or against the state.

If a corners agree and d disagree, the scan likelihood is:

`L(i,h) = 0.8^a * 0.1^d`.

We multiply each prior belief by this likelihood and normalize over every intersection and heading. We then mix in a small uniform distribution:

`b_new(i,h) = (1-epsilon) * b_old(i,h)*L(i,h)/Z + epsilon/(4mn)`

where `Z = sum over all states of b_old*L` and `epsilon = 0.000001`.

Thus, replacing a mismatch with a match makes a hypothesis eight times more likely relative to its previous score. A mismatch reduces support without permanently eliminating the state. The uniform recovery component also allows states with extremely small weights to regain support, while keeping the distribution normalized.

The 0.8/0.1 values express an assumed 80% correct classification rate with equal chances for the other two landmark colours. They are initial model choices, not an empirically measured confusion matrix. Multiplication also assumes corner readings are conditionally independent given the pose.

Only the **current scan** determines this update's likelihood; previous raw scans are not scored again. Earlier evidence is retained through the prior belief, as in a recursive Markov filter.

## Q5. What is our exploration strategy and why was it chosen?

After acquiring the first intersection, we scan, update the distribution and test whether both position and heading are sufficiently clear. If they are not, the robot repeats the action pattern **right, left, straight**. For a right or left action, it turns 90 degrees at the current intersection, updates heading beliefs, and then drives toward the next intersection. For a straight action, it drives without the preliminary turn.

Each completed trip updates the motion beliefs. An ordinary yellow arrival moves probability mainly to the neighbouring intersection; a red-boundary turnaround returns it to the previous intersection with a reversed heading. The next scan then supplies new landmark evidence.

This is a simple, reproducible way to sample streets in both axes and encounter different landmark sequences, instead of staying on the initial street. It does not select actions by information gain or guarantee complete map coverage. An attempt is bounded to 24 scans; unresolved or failed attempts return control to the mission's bounded recovery process.

## Q6. How are beliefs shifted at a boundary, and what motion assumptions support this?

When red is detected, the robot turns 180 degrees using gyro feedback, reacquires the street and follows it back to yellow. The motion model assumes this is the **same intersection it just left**, so a successful boundary return changes heading without moving the discrete position to a different cell.

For each candidate state `(x,y,h)`, we check whether its next grid step would leave the map. Only outward-facing boundary states explain a red-and-return event. Their probability is distributed at the original intersection as follows:

- 96% to the reversed heading `(h+2) mod 4`.
- 2% to `(h+1) mod 4`.
- 2% to `(h+3) mod 4`.

Prior hypotheses at interior intersections or pointing inward contribute no transition mass to the red-return update. The reversed destination heading is inward-facing and does receive probability. We normalize the accumulated destination mass and apply the same small uniform recovery mixture. Destinations unsupported by any eligible boundary hypothesis then receive only the recovery floor. A second red detection during recovery fails the trip instead of triggering another untracked turnaround.

For comparison, a direct yellow arrival rejects outward-facing edge states. Each eligible state sends 94% of its mass one cell forward and 6% to its original cell, preserving heading. Beliefs never wrap across opposite map edges. If no state can explain the reported outcome, the update fails without replacing the existing distribution.

The boundary model assumes genuine red detection, a successful return to the immediately previous intersection, and no unobserved displacement to another cell during recovery. The 96/2/2 turnaround probabilities and 94/6 forward probabilities are initial motion assumptions requiring physical validation.

## Q7. How are beliefs shifted for a 90-degree turn at the same intersection?

The main turn transition keeps the intersection fixed and redistributes heading probability. For a right turn, the intended heading is `(h+1) mod 4`; for a left turn, it is `(h+3) mod 4`.

For either command, each original state's probability contributes:

- 96% to the intended heading.
- 2% to the original heading, representing no turn.
- 2% to `(h+2) mod 4`, representing a half turn.

We accumulate contributions into a separate array, normalize and apply the uniform recovery mixture. Using a separate array prevents an update from overwriting values that are still needed.

This assumes the axle is positioned over the intersection and the robot rotates in place without changing grid cells. Gyro feedback controls the physical turn, while the probability spread allows coarse residual heading errors. The small uniform recovery mixture is separate from the same-intersection motion model. These error probabilities have not been established by physical measurements.

## Q8. How do we decide that localization has succeeded?

We check position and heading separately. First, we sum the four heading beliefs at each intersection to obtain its position probability. We select the best intersection and its runner-up. At the best intersection, we divide each heading belief by that intersection's total to obtain conditional heading probabilities, then select the best heading and its runner-up.

**Both comparisons must satisfy the same rule:**

- The best probability is at least ten times the runner-up; **or**
- The best probability is greater than 0.70 and at least five times the runner-up.

This prevents a clear position with an ambiguous orientation from ending localization. There is no fixed minimum number of valid corners and no requirement for consecutive confirming scans. A sufficiently informative scan can finish an attempt, while an ambiguous result leads to further exploration. The attempt fails if it remains unresolved after 24 scans.

After success, navigation follows a horizontal-first, then vertical Manhattan route to the target, using colour and gyro feedback without further landmark scans or belief updates. A failed turn/trip or unexpected boundary invalidates the pose and can trigger relocalization. The mission allows two recovery retries after the initial attempt. A silently wrong pose can still escape detection if subsequent sensor events appear consistent.

## Q9. What robot design and sensor-placement choices support the implementation?

Our controller is designed for a differential-drive robot with independently driven wheels: motor C is treated as the left wheel and B as the right. This supports straight driving, reverse motion, and in-place 90-degree and 180-degree turns required by the grid and boundary recovery.

The code assumes a downward-looking colour sensor mounted ahead of the drive axle. The same sensor reads streets, intersections, landmarks and boundaries. On arrival, yellow is initially beneath the sensor; before scanning or turning, the controller advances by the sensor-to-axle offset to place the axle over the intersection. Rotating then sweeps the forward sensor around the corner regions. The colour sensor uses input 3 and the gyro uses input 2. Gyro feedback supplies relative turn angles and heading correction, while motor encoders bound travel distance.

The configured offset is **0.27 wheel revolutions**. It remains an assumed value until checked against the actual forward sensor offset and wheel diameter; the corresponding geometric calculation is `sensor offset / (pi * wheel diameter)`. The repository does not establish chassis dimensions, sensor height or shielding, so those physical details should be added only if confirmed on the assembled robot.

## Q10. Which parameters most affect localization?

The implementation and software tests identify several sensitive groups of parameters. We have not established a ranked list through controlled hardware experiments.

| Parameter group | Why it matters |
|---|---|
| Colour calibration and class separation | Incorrect landmark colours can favour a wrong pose. The classifier's nearest/runner-up squared-distance ratio is 0.85. |
| Sensor-to-axle offset and centring | The 0.27-revolution advance determines the sensor's scan path. Incorrect geometry can systematically sample the wrong regions. |
| Scan reference and angular accuracy | Scans use 10-degree spacing and corner sectors within 25 degrees of their expected diagonal. Return/turn checks use a 3-degree tolerance. A wrong corner order directly corrupts orientation evidence. |
| Voting thresholds | Four-of-five local agreement, at least three agreeing angular samples, and 70% building-vote agreement balance missing data against false colours. |
| Motion control | Street power 10, heading gain 1.5, turn gain 0.6 and turn power limits affect drift, overshoot and reaching the intended marker. |
| Probabilistic model and stopping rules | The 0.8/0.1 likelihoods, motion uncertainty and best/runner-up thresholds control how quickly the filter concentrates and how readily it accepts an incorrect hypothesis. |

Synthetic sensitivity tests reinforce the importance of correct landmark readings. With 10% missing corners, all 6,000 runs localized correctly. With a 10% independent wrong-colour probability, 5,806 of 6,000 were correct and 194 returned an incorrect pose. These imposed noise rates are test conditions, not measurements of our sensor.

## Q11. Can external factors affect localization?

Yes. Changes in illumination, shadows and sensor-to-map distance can alter RGB values and therefore classification. HSV features and repeated readings improve robustness but do not guarantee correct readings after a large environmental change. Calibration should reflect the mounted sensor and operating conditions.

The reader supports two distinct sensor formats: NXT readings include ambient correction, while EV3 RGB-RAW readings have no separate ambient channel and receive no ambient subtraction. Calibration files must match the attached sensor.

Battery condition, wheel slip, surface friction, map flatness and print scale can also affect motion, centring and scanning geometry. Gyro feedback, encoder limits, braking and bounded recovery reduce these effects without eliminating them. Communication errors or competing EV3 programs can also disrupt readings. Repeated sampling helps random noise more than persistent systematic errors.

## Q12. On average, how many intersections are visited before localization?

In the **ideal robot-free simulation of the current implementation on Map1**, localization required an average of **2.45 intersection scans** across all 60 starting intersection-heading combinations. All 60 runs returned the correct final position and heading.

| Number of scans | Starting poses |
|---|---:|
| 2 | 35 |
| 3 | 23 |
| 4 | 2 |

The average is `(35*2 + 23*3 + 2*4) / 60 = 2.45`; the median is two and the range is two to four.

This count includes the first scanned intersection and repeated visits after boundary returns, so it is a count of scan visits, not distinct intersections. It excludes travel before acquiring the first intersection. The corresponding mean number of exploration trips after the first scan is 1.45.

With 10% missing corners, the simulated average increased to approximately 2.57 scans, with all 6,000 runs correct. No physical average is established by the saved evidence, so **2.45 should be reported as a simulation result**, not a measured robot result.

## Q13. Do some starting locations make localization harder?

Yes. A single colour pattern may occur at several intersections or match after rotation, making position or heading ambiguous. On Map1, all 15 intersections fall into only three four-corner pattern classes when rotations are allowed. Consequently, one initial scan cannot uniquely determine position. These individual patterns are not rotationally self-symmetric: the ambiguity is between different locations, including rotated patterns at those locations. Missing corners remove evidence that might otherwise separate hypotheses.

On Map1, starting heading also affects which landmarks and boundaries the fixed right-left-straight policy encounters. In the ideal simulation, starts at `(0,4)` and `(2,4)` averaged three scans over their four headings. The two slowest poses were `(0,4)` facing right and `(2,4)` facing up, each requiring four scans. Starting at `(2,2)` required two scans for every heading. Coordinates are zero-based from the upper-left.

Boundaries have two effects: a return can revisit an already scanned intersection, delaying acquisition of a new landmark pattern, but the red event also rules out interior/inward-facing hypotheses. Therefore, edge starts are not uniformly worse; difficulty depends on the starting pose, landmark sequence and exploration actions together.

## Optional bonus feedback - suggested answers for team review

The following are editable suggestions, not claims inferred about the team's personal experience. Keep or revise them to match that experience.

1. **Main steps of localization: Yes.** Implementing sensing, motion prediction, measurement updates, normalization and a stopping criterion connected the lecture algorithm to an executable control loop. More labelled end-to-end traces would make debugging easier.
2. **Real-world sensors: Yes.** Calibration, ambiguous colours and invalid replies showed why a raw reading needs validation and repeated measurements. A standard sensor-height and lighting checklist would improve repeatability.
3. **Real-world motors and motion: Yes.** The controller highlighted the need for gyro feedback, axle centring, braking and bounded recovery. More structured turn and one-street trials would help distinguish mechanical errors from localization errors.
4. **Probabilities for position belief: Yes.** Maintaining all position-heading hypotheses showed how individually ambiguous observations become useful through motion and recursive updates.
5. **Managing uncertainty: Yes.** Unknown readings, likelihood weights, motion uncertainty and recovery demonstrated the difference between missing evidence and contradictory evidence. Measured sensor and motion error rates would improve the model.
6. **Confidence about a more complex robot: Somewhat.** The predict-update structure transfers to larger problems, but a real-world system would need a richer state/motion model, better quantified uncertainty and substantially more physical validation.
7. **Other comments:** Robot-free tests were useful for separating algorithm/control-flow faults from hardware behaviour. More opportunities for repeatable hardware trials would help connect simulated performance to physical reliability.

## Verification and source notes

The source PDF is [C85_Project2_Localization.pdf](/cmshome/wangs852/Desktop/localisation/C85_Project2_Localization.pdf). Q1-Q13 appear on printed pages 5-9; optional feedback is on the final page, labelled 14.

Implementation references:

- [Likelihood and measurement update](/cmshome/wangs852/Desktop/localisation/EV3_Localization.c:678).
- [Turn and completed-trip motion models](/cmshome/wangs852/Desktop/localisation/EV3_Localization.c:750).
- [Confidence checks and exploration loop](/cmshome/wangs852/Desktop/localisation/EV3_Localization.c:860).
- [Mission belief reset and retries](/cmshome/wangs852/Desktop/localisation/EV3_Localization.c:1197).
- [Intersection corner voting and boundary recovery](/cmshome/wangs852/Desktop/localisation/EV3_Localization.c:494).
- [Calibration, HSV classifier and sample voting](/cmshome/wangs852/Desktop/localisation/EV3_Localization.c:1447).
- [Sensor port definitions](/cmshome/wangs852/Desktop/localisation/EV3_Localization.h:101).
- [Verification methods and physical-test limitations](/cmshome/wangs852/Desktop/localisation/verification/README.md).

Simulation CSVs:

- [Ideal: all 60 starting poses](/cmshome/wangs852/Desktop/localisation/logs/pdf_answers_ideal_20261022.csv).
- [10% missing corners: 6,000 runs](/cmshome/wangs852/Desktop/localisation/logs/pdf_answers_missing10_20261022.csv).
- [10% wrong colours: 6,000 runs](/cmshome/wangs852/Desktop/localisation/logs/pdf_answers_noise10_20261022.csv).

The simulator compiles the production localization functions, supplies landmarks using an independent PPM reader, and replaces robot movement/sensing with a discrete simulated robot. It checks the returned pose against simulated ground truth. Ideal tests use perfect discrete motion and complete, correct corner observations; synthetic noise tests alter corner observations directly and do not simulate raw RGB or mechanical scanning.

Reproduce the cited experiments with:

```sh
python3 verification/simulate_localization.py
python3 verification/simulate_localization.py --unknown 0.10 --trials 100 --seed 1
python3 verification/simulate_localization.py --noise 0.10 --trials 100 --seed 1
```

Fresh software checks also passed for the classifier/filter, colour readers, sensor selection, gyro reader and intersection sweep. [Boundary recovery](/cmshome/wangs852/Desktop/localisation/logs/red_boundary_tests_20261022T182316.494514Z.md) passed 64/64 cases; [mission recovery](/cmshome/wangs852/Desktop/localisation/logs/mission_recovery_tests_20261022T182316.731509Z.md) passed 20/20 cases, including 900 full scripted routes. Scripted tests establish software behaviour, not physical navigation accuracy.

One implementation detail outside the numbered questions: `beep()` currently returns success with its tone playback commented out. The application prints completion messages, but the report should not claim that an audible success tune is currently enabled.
