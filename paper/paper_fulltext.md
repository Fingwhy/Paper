# FACTR 2: Learning External Force Sensing for Commodity Robot Arms Improves Policy Learning

> 全文文本由 arXiv 官方 PDF (arXiv:2606.12406v2) 自动提取，保留原始分页与图表标题。



<!-- ===================== PDF Page 1 ===================== -->

FACTR 2: Learning External Force Sensing for
Commodity Robot Arms Improves Policy Learning
Steven Oh∗1,2
Jason Jingzhou Liu∗†1
Tony Tao∗1
Philip Han1
Kenneth Shaw1
Satoshi Funabashi2
Ruslan Salakhutdinov1
Deepak Pathak1
1Carnegie Mellon University
2Waseda University
∗Equal contribution
Abstract: Contact-rich manipulation requires force sensitivity, but many robot
arms lack dedicated force sensors due to their high cost. We present Neural Ex-
ternal Torque Estimation (NEXT), a data-driven method that estimates exter-
nal joint torques without needing any dedicated force sensors. NEXT trains in
1 minute from only 10 minutes of free-motion data, yet achieves estimates com-
parable to dedicated joint-torque sensors. NEXT enables force-feedback teleop-
eration on low-cost arms and improves policy learning through Force-Informed
Re-Sampling Training (FIRST), which up-samples pre-contact and contact seg-
ments during behavior cloning. Across five long-horizon tasks, FIRST outper-
forms prior force-aware policies by over 17% in task progress. Together, NEXT
and FIRST bring force-aware teleoperation and policy learning to off-the-shelf
robots without additional sensing hardware. Video results and code are available
at https://jasonjzliu.com/factr2
Figure 1: Overview of our approach. (a) Neural External Torque estimation (NEXT) produces
high quality joint torque estimates using only 10 minutes of data without dedicated force sensors or
explicit system-identification, enabling force-feedback teleoperation on low-cost arms, such as the
Piper, YAM, and Nero. (b) Force-Informed Re-Sampling Training (FIRST) uses learned external
torque estimates to segment demonstrations into free-space, pre-contact, and contact phases, then
up-samples contact-relevant segments during training to improve policy performance.
1
Introduction
Many everyday manipulation tasks, such as insertion, fastening, and deformable-object handling,
require precise force-aware control that humans naturally rely on. In robotics, these subtle contact
interactions are difficult to infer reliably from vision alone, making force sensing an essential capa-
bility. Dedicated force-torque (FT) sensors are typically available only on expensive platforms such
as Franka, Flexiv, or KUKA arms. Many low-cost robot arms lack built-in force sensing, limiting
their ability to provide force feedback during data collection and preventing policies from lever-
aging the force information needed for robust, contact-aware manipulation. Retrofitting platforms
with end-effector force sensors remains challenging and expensive, sometimes costing more than
†Correspondence to: Jason Jingzhou Liu, liujason@cmu.edu
arXiv:2606.12406v2  [cs.RO]  12 Aug 2026



<!-- ===================== PDF Page 2 ===================== -->

the arms themselves. Conventional piezoresistive FT sensors mounted between the arm and end
effector rely on multiple strain gauges attached to precision-machined structures, making them ex-
pensive and bulky [1, 2]. Recent capacitive sensors such as CoinFT [3] and magnetic tactile sensors
such as ReSkin [4] offer lower-cost alternatives, but their soft sensing materials are not designed to
withstand the larger force magnitudes often experienced at the end effector [5].
Instead, we present Neural External Torque estimation (NEXT), a data-driven method for estimating
external joint torque, producing high-quality estimates even on low-cost robot arms. NEXT requires
only 10 minutes of contact-free trajectories and 1 minute of training to learn a neural network that
predicts the torque required for free-space motion. At deployment, NEXT estimates external torque
as the residual between measured motor torque, inferred from motor current, and the predicted free-
space torque. Importantly, NEXT does not require dedicated force-torque sensors during either data
collection or deployment. We show that NEXT produces signals that closely match dedicated joint-
torque sensor measurements on high-end robot arms, while outperforming prior estimation methods.
The learned external force signal enables two capabilities: (1) Force-feedback teleoperation on non-
sensorized, low-cost arms (e.g., AgileX Piper, YAM), achieving performance comparable to sen-
sorized systems [6, 7]; and (2) Force Informed Re-Sampling Training (FIRST), a behavior cloning
method that uses external force estimates to segment demonstrations into free-space, pre-contact,
and contact phases, and up-samples the latter two to improve policy success.
FIRST is motivated by the observation that policy failures concentrate not in free-space motion, but
in the brief pre-contact intervals requiring precise alignment as well as in contact phases requiring
controlled interaction and fine corrections. We evaluate FIRST on five long-horizon, contact-rich
tasks, where it consistently improves success rates over prior baselines, with gains in alignment,
recovery, and contact-phase performance.
With NEXT and FIRST, we call our combined system FACTR 2. It generalizes FACTR [6] beyond
arms with dedicated force sensors, enabling both force-feedback teleoperation and force-aware pol-
icy learning on commodity robot arms. We believe FACTR 2 democratizes force sensing for any
robot arm as we evaluate this capability on platforms spanning low-cost arms such as the Piper
($2,500) and high-end arms such as the Franka ($30,000). We summarize our contributions:
• Data-driven external torque estimation without force sensors. We present NEXT, a
neural method for estimating external joint torque without dedicated FT sensors or robot-
specific system identification. NEXT trains a recurrent model on only 10 minutes of free-
motion data and produces higher-quality estimates than prior methods.
• Force-feedback teleoperation on low-cost arms. We use the estimated force signal to
enable bilateral teleoperation on non-sensorized low-cost arms, including Piper and YAM,
achieving feedback quality comparable to sensorized systems.
• Force-Informed Re-Sampling Training. We propose FIRST, a behavior cloning method
that uses external torque estimates to identify pre-contact and contact phases in demonstra-
tions, then up-samples these phases during training to improve policy performance.
2
Related Works
2.1
Sensorless External Force Estimation
Estimating external forces without dedicated sensors usually involves modeling robot dynamics and
interpreting the residuals between predicted and observed signals as external contact. Traditional
approaches based on analytical modeling and system identification [8, 9, 10] can be effective, but
they require substantial manual engineering effort and often struggle with actuator nonlinearities like
dead zones [11] and torque ripple [12]. Learning-based methods have emerged to address these lim-
itations. Supervised approaches learn from paired proprioceptive and real world [13] or simulation
[14] contact data , but require dedicated data collection and labeling. Unsupervised methods such as
autoencoder-based anomaly detection [15] can identify deviations from manipulator free motion, but
are primarily suited for contact detection rather than continuous force estimation. Inverse-dynamics
learning has been explored for both individual actuators [12] and complex high-end platforms such
as the da Vinci system [16], but has not been demonstrated broadly on low-cost robots.
2



<!-- ===================== PDF Page 3 ===================== -->

2.2
Imitation Learning with Force Information
Recent imitation learning methods incorporate force or torque information in several ways. One
line of work uses force-related signals as an additional policy input. For example, TA-VLA [17]
uses joint effort information, BiACT [8] uses joint torque, and CompACT [18] conditions the pol-
icy on end-effector force. ForceVLA [19] further introduces a mixture-of-experts architecture for
context-aware routing across modality-specific experts. Some methods, such as TA-VLA [17] and
FoAR [20], use auxiliary objectives to encourage contact-aware representations. A second line
of work combines behavior cloning with hybrid force-position or impedance control at inference
time [21, 22, 23]. These methods predict both motion and force targets, which are then tracked by
a low-level controller to enforce the desired interaction behavior. Finally, FACTR [6] uses force
information during training to improve policy generalization to unseen objects. Our method, FIRST,
uses external torque not only as policy information, but to infer contact phases and use them to re-
shape the training distribution. Our ablations further show that pre-contact samples are often more
valuable than in-contact samples, providing an empirical guideline for force-aware data reweighting.
2.3
Policy Learning with Dataset Reweighting
Policy training dynamics is strongly shaped by how data is used during optimization. In reinforce-
ment learning, prior work has used value and advantage-based weighting to emphasize actions that
produce higher returns. AWR [24] weights actions by exponentiated advantages, CRR [25] uses
a learned critic to filter or weight actions, and AWAC [26] applies advantage-weighted updates to
bridge offline data and online fine-tuning. In imitation learning, data reweighting has also been
explored. Re-Mix adapts DoReMi-style [27] Group DRO for robotics [28], while DataMIL uses
data model estimators [29] for target-aware selection [30]. DemInf scores state-action mutual in-
formation to favor diverse, predictable actions [31], and SCIZOR combines progress prediction
and embedding clustering to prune transitions [32]. Other methods leverage learned value func-
tions for filtering: GVL uses vision-language models for value estimation [33], and SARM trains a
stage-aware reward model [34]. Finally, rollout-based methods optimize closed-loop performance
directly. Demo-SCORE filters data using a success classifier trained on rollouts [35], and CUPID
uses influence functions to estimate how individual demonstrations affect expected returns [36].
3
Background
External joint torque τext is the joint-space torque induced by physical interactions with the environ-
ment, excluding the torque required to move the robot in free space. This signal is directly available
on manipulators with dedicated force-torque sensing, but many arms do not have those sensors.
In the absence of dedicated force sensing, external joint torque can be estimated from motor current
measurements and a model of the robot dynamics. Motor current is readily provided on most robot
arms, and is approximately related to motor torque through τm = KIm, where Im is the measured
motor current and K is the torque constant. Given measured motor torque τm and a known robot
dynamics model, the external joint torque can be taken as
τext = τm −τf,
τf = M(q)¨q + C(q, ˙q) ˙q + g(q) + τother
(1)
Here, τf denotes the torque required to realize the observed motion in free space, i.e., in the absence
of external contact. Computing τf therefore requires an accurate model of the robot mass matrix M,
Coriolis and centrifugal terms C, gravity term g, and any other contributing terms τother.
However, model-based external torque estimation is highly sensitive to errors in estimating τf. In
practice, accurately identifying the inertial, gravitational, and friction parameters of a robot is diffi-
cult, especially when only motor-side measurements are available [37, 38]. Moreover, the residual
τm −τf captures not only contact-induced torque, but also unmodeled actuator and transmission
effects, including nonlinear friction, stiction, backlash, hysteresis, temperature-dependent drive be-
havior, sensing noise, torque ripple, deadzones, and saturation [39, 40, 41]. Because many of these
effects are history-dependent and only partially observable from instantaneous joint state, free-space
modeling errors can limit purely model-based sensorless force estimation.
3



<!-- ===================== PDF Page 4 ===================== -->

4
Learning External Force Estimation
We present NEXT, Neural External Torque Estimation, a data-driven method for estimating exter-
nal joint torque τext. Importantly, NEXT does not require dedicated force-torque sensors for neither
data collection nor deployment. Instead of computing free-space torque τf from model-based in-
verse dynamics, we learn a free-space inverse dynamics neural network directly from joint states,
predicting the motor torque that would be observed in the absence of external contact:
ˆτf = fθ(x),
ˆτext = τm −ˆτf.
(2)
where fθ is a neural network parameterized by θ, and x is a history of proprioceptive observations.
At deployment time, we estimate external joint torque ˆτext by subtracting the predicted free-space
torque ˆτf from the measured motor torque τm = KIm, illustrated in Figure 2. As shown in Section
7 and Appendix A.2, NEXT yields higher quality external torque estimates against prior methods.
To train the free-space inverse-dynamics model, we collect a dataset of robot arm trajectories exe-
cuted without external contact, Dfree = {(xi, τm,i)}N
i=1, where τm,i is the measured motor torque
at timestep i. The input xi is constructed from a temporal history of proprioceptive quantities,
xi = [qi−H:i, ˙qi−H:i, ∆qd,i−H:i] ,
(3)
where H is the history length, q denotes joint position, ˙q denotes joint velocity, and ∆qd,i is the
difference between the commanded joint position target qd,i and the current joint position qi.
Because all trajectories in Dfree are collected without external contact, the measured motor torque
equals the free-space torque. Thus, we use τm,i as the supervision target for learning τf. In practice,
the dataset can be collected from approximately 10 minutes of free-space robot motion, generated
either by teleoperation, motion planning, or replay of known free-space trajectories. Thus, this
procedure is easy to adopt on new robot platforms, as it does not require force-torque sensing,
payload calibration, or manual system identification.
Figure 2: External Force Estimation Deployment.
At deployment time, we first obtain the measured
joint torque from multiplying each joint’s measured
current by its torque constant K. We then use an
LSTM trained on free-space data to estimate free-
space torque, which is then subtracted from the mea-
sured joint torque to obtain external joint torque.
We instantiate fθ as an LSTM-based se-
quence model, which maps the propriocep-
tive history xi to a predicted free-space
torque ˆτf,i. The use of temporal history al-
lows the model to capture effects that are dif-
ficult to represent with an instantaneous rigid-
body dynamics model, such as hysteresis,
actuator dynamics, frictional memory, and
controller-dependent tracking behavior. The
model is trained with an L2 regression loss,
L(θ) = 1
N
N
X
i=1
∥fθ(xi) −τm,i∥2
2
(4)
After training, the model predicts the free-space torque ˆτf, allowing external torque to be estimated
as the residual τm −ˆτf. Training is lightweight, requiring only 1 minute on an NVIDIA RTX 3090.
Details on the model architecture and hyperparameters are provided in Appendix A.
5
Force-Informed Re-Sampling Training
To leverage external joint torque for improved behavior cloning, we introduce FIRST, Force-
Informed Re-Sampling Training. FIRST is motivated by the observation that failures in contact-rich
tasks often occur near contact: pre-contact phases require precise alignment, while contact phases
demand controlled interaction and fine corrective actions. FIRST therefore re-weights the train-
ing data mixture to sample pre-contact and contact segments more frequently, focusing learning on
the most failure-prone portions of the trajectory. In Section 7, we show that FIRST improves task
success by over 17% and reduces validation loss on pre-contact and contact phases compared to
standard behavior cloning, explaining its downstream performance gains.
4



<!-- ===================== PDF Page 5 ===================== -->

Figure 3: FIRST is evaluated on five long-horizon, contact-rich tasks. Each task comprises multiple
stages, many requiring precise alignment or fine-grained, force-sensitive adjustments. Please see
videos of these tasks at https://jasonjzliu.com/factr2
Problem Statement. We consider a policy that, at timestep t, conditions on an observation ot =
(I1:M
t
, qt, ˆτext,t), consisting of M camera images, joint position qt, and the estimated external
joint torque ˆτext,t from Section 4. The policy predicts a chunk of future joint position targets,
ˆat:t+k ∼πθ(· | ot), where at:t+k denotes a corresponding action chunk.
We train the policy by behavior cloning on expert demonstrations D = {(ot, at:t+k)} . We instanti-
ate πθ as a flow-matching policy parameterized by a conditional velocity field vθ. Let x1 = at:t+k
be an expert action chunk, x0 ∼N(0, I) be noise, and xα = (1 −α)x0 + αx1 for α ∼U(0, 1).
The policy is trained with a velocity prediction objective [42], where vθ predicts the flow from noise
x0 to the expert action chunk x1:
LFM(θ) = E
h
∥vθ(xα, α, ot) −(x1 −x0)∥2
2
i
.
(5)
Segmenting Trajectories into Contact Phases. To begin, FIRST labels each datapoint in the
demonstration dataset as free motion, pre-contact, or contact. Contact corresponds to timesteps
where the robot is physically interacting with the environment, pre-contact corresponds to the one-
second window before contact onset, and all remaining timesteps are labeled as free motion.
We identify contact from the estimated external joint torque ˆτext,t using the scalar contact score ft =
∥ˆτext,t∥1. To avoid noisy transitions, we use hysteresis thresholding: the binary contact indicator ct
turns on when ft ≥Thigh, turns off when ft ≤Tlow, and otherwise retains its previous value.
Contact onsets are timesteps where the label ct switches from non-contact to contact,
Tonset = {t | ct−1 = 0, ct = 1}
(6)
Given a control frequency F, we assign the phase label st by marking contact timesteps as Contact,
the F timesteps before each contact onset as Pre-Contact, and all other timesteps as Free Space:
st =



Contact (C),
ct = 1,
Pre-Contact (PC),
∃tonset ∈Tonset : tonset −F ≤t < tonset,
Free Space (F),
otherwise.
(7)
Up-Sampling Pre-Contact and Contact Phases in Training. After assigning each datapoint a
phase label st ∈{Free Space, Pre-Contact, Contact}, FIRST modifies only the sampling dis-
tribution used for batch construction. We assign phase-dependent sampling weights wF, wPC, and
wC to free-space, pre-contact, and contact datapoints, respectively, with wPC, wC > wF. During
training, datapoints are sampled with probability pt = w(st)/ PN
j=1 w(sj), where w(st) denotes
the weight associated with the phase label of datapoint t. This weighted random sampling increases
the fraction of pre-contact and contact examples in each minibatch, emphasizing trajectory segments
where precise alignment and force-sensitive corrections are most critical.
5



<!-- ===================== PDF Page 6 ===================== -->

6
Experiments
6.1
External Force Estimation and Force-Feedback Teleoperation
We evaluate external torque estimation on a Franka arm in both contact and free-space settings.
We use the Franka because it has built-in joint torque sensors and a factory-calibrated external
torque estimate, providing a high-quality reference which we use as ground truth during contact.
For contact, we collect 5 minutes of held-out data, unseen during training, while a human applies
external forces to the robot. We report the L1 error averaged across joints between the ground-truth
sensor signal and estimates from NEXT (ours), FILIC [22], and a disturbance observer (DO) [43].
For free-space motion, where the true external torque should be zero, we collect 5 minutes of held-
out data without contact and use zero torque as ground truth, treating the Franka sensor signal as
another baseline. Baseline details are provided in Appendix A.4. We repeat the free-space evaluation
on the AgileX Piper, a low-cost arm without force sensors, again using zero torque as ground truth.
Since the Piper lacks external torque sensing, we cannot repeat the contact evaluation; instead, we
assess contact performance through a force-feedback teleoperation user study.
Force-Feedback Teleoperation User Study.
We evaluate whether the learned exter-
nal torque estimate provides useful teleoperation feedback.
In a user study with 20
participants,
users perform a wiping task on the Franka arm,
as shown in Figure 4,
Figure 4: User Study. Our external force esti-
mate (using NEXT) improves teleoperation over
baselines and remains comparable to FACTR
Teleop, which relies on dedicated external force
sensors. Lower joint torque applied corresponds
to less unnecessary exertion.
with 5 feedback conditions:
no feedback,
disturbance-observer
based
feedback
(DO),
leader-follower position-position feedback [44]
(PP), FACTR Teleop using Franka’s built-
in torque sensors, and FACTR Teleop with
NEXT’s external torque estimates.
Note that
both FACTR Teleop variants implement the fol-
lowing force-feedback control law [6]:
τfeedback = Kf,pτext
(8)
where τfeedback is joint torque applied to the
leader arm, and Kf,p scales the external joint
torque τext sensed by the follower arm. In this
study, τext either comes from Franka’s factory
external torque estimate or comes from NEXT.
We collect 1–5 ease-of-use ratings and measure
the applied joint torque. We repeat the study on
the Piper arm. Details about the baselines are
described in Appendix A.5.
6.2
Imitation Learning with Force Input
Experiment Settings. We evaluate FIRST against baselines on five long-horizon, contact-rich ma-
nipulation tasks, shown in Figure 3. Detailed task descriptions are provided in Appendix C.1. For
each task, we collect 250 demonstrations on a bimanual Piper setup with four RGB cameras: two
wrist cameras and two overhead cameras. Each policy is evaluated over 20 rollouts per task. We
report task progress, with scoring criteria defined in Appendix C.2.
Policy Details and Baselines. We compare FIRST with four baselines while keeping the policy ar-
chitecture and behavior cloning objective fixed. All methods use a flow-matching policy trained for
20 epochs, with an architecture adapted from Multi-modal Diffusion Transformer [45] and an action
chunk size of 30. We also repeat the experiments with Action Chunking Transformer (ACT) [46].
The action head of each policy is randomly initialized and trained from scratch. Implementation
details for both are provided in Appendix B. We compare the following variants:
• Base Policy: Policy takes in images and robot joint positions as inputs, with no force information.
• Base Policy + Torque: Base policy with the learned external torque estimate as additional input.
• FACTR [6]: Uses external torque and blurs input images during training with a noise schedule.
• TA-VLA [17]: Uses an auxiliary objective to reconstruct the input external torque.
• FIRST (Ours): Uses learned external torque as input and up-samples pre-contact and contact data
during training. We ablate up-sampling ratios in Appendix D.4.
6



<!-- ===================== PDF Page 7 ===================== -->

Motion Type
External Torque Method
L1 Error (Nm)
Contact
External Sensor (GT)
0.000 ± 0.000
FILIC [22]
4.395 ± 1.531
Disturbance Observer [43]
1.471 ± 0.761
NEXT (Ours)
0.547 ± 0.348
Free Space
Free Space (GT)
0.000 ± 0.000
FILIC [22]
2.460 ± 1.358
Disturbance Observer [43]
2.429 ± 1.298
External Sensor
0.449 ± 0.208
NEXT (Ours)
0.414 ± 0.278
Table 1:
Average L1 joint torque error
on the Franka in free-space and contact
settings.
Free-space errors are measured
against zero external torque, while con-
tact errors are measured against the Franka
built-in external torque estimate.
NEXT
achieves the lowest error in both settings.
Figure 5: Left: In free space, the external joint torque
should remain zero. Our method produces an esti-
mate that is less noisy than the external sensor and re-
mains near zero, while FILIC and the Disturbance Ob-
server drift away. Right: During contact, our estimate
closely tracks the external sensor, whereas FILIC and
the Disturbance Observer deviate substantially.
7
Results
In our external force estimation experiments, we ask: (i) How do NEXT’s external joint torque
estimates compare to ground-truth sensor measurements and baselines? (ii) How effective is NEXT
for force-feedback teleoperation compared to alternative feedback methods?
In our imitation learning experiments, we ask: (i) How does FIRST compare to other baselines
leveraging force? (ii) Which contact phases should FIRST up-sample for best performance?
NEXT predicts external joint torque comparable to dedicated sensors. We present quantitative
results for NEXT on the Franka system in Table 1. Under external contact, NEXT closely matches
the Franka external joint torque sensor, achieving an average L1 error of only 0.547 Nm, which is
87.6% lower than FILIC and 62.8% lower than the disturbance observer. This trend is also shown
qualitatively on the right side of Figure 5: NEXT tracks the sensor signal closely, while the baselines
deviate substantially, especially during contact transitions.
In free space, where the true external torque should be zero, we treat the Franka’s external torque
sensor signals as an additional baseline; NEXT achieves the lowest error, even outperforming this
dedicated sensor. As shown on the left side of Figure 5, its estimate is less noisy and remains closer
to zero than the sensor. This can be attributed to NEXT learning hardware-specific effects directly
from data collected on the hardware itself, yielding high-quality force estimates.
We observe similar results on the AgileX Piper arm, where NEXT achieves an average joint L1
error of only 0.018 Nm, substantially lower than the baselines; full results are provided in Table 5
in Appendix A.6. We also ablate the NEXT model architecture in Appendix A.2, showing that the
LSTM outperforms both GRU and MLP variants.
NEXT enables high-quality force-feedback teleoperation on low-cost arms. In the Franka user
study, participants rated NEXT-based force feedback as easier to use than the baselines, shown in
Figure 4 (a). NEXT also reduced the force required to complete the task, performing comparably
to FACTR Teleop, which uses dedicated joint torque sensors, seen in Figure 4 (b). Lower applied
force indicates greater force awareness and less unnecessary exertion. On the Piper arm, which
lacks dedicated force sensors, NEXT also outperforms the baselines, as detailed in Appendix A.6.
Overall, these results show that NEXT provides high quality force feedback without needing force
sensors, comparable to sensor-based methods.
FIRST leads to improved policy performance. In Figure 6, we present the main qualitative re-
sults for FIRST against baselines for the flow matching policy, with ACT results shown in Appendix
D.2. FIRST achieves the highest task progress across all five real-world tasks. The improvement is
especially clear in long-horizon, multi-stage tasks, such as NIST Belt, where successful execution
requires reliable transitions between free motion, pre-contact alignment, and contact-rich manipu-
lation. Qualitatively, we observe that FIRST produces more robust recovery behaviors before key
contact events, such as fine-grained realignment motions before insertion, wrapping, or screwing.
7



<!-- ===================== PDF Page 8 ===================== -->

LEGO Assembly
NIST Belt
NIST Insertion
Tool Clean Up
Cap Screwing
0%
20%
40%
60%
80%
100%
Task Progress
Base Policy
Base Policy + Torque
FACTR
TA-VLA
FIRST
Figure 6: Task progress across five contact-rich manipulation tasks using flow matching policy.
0
20000
40000
10−1
100
101
Validation Loss
Base Policy + Torque
0
20000
40000
FIRST (Ours)
Contact
Pre-contact
Free motion
Training Step
Figure 7: Default sampling yields higher valida-
tion loss on pre-contact and contact phases. By
upsampling these phases during training, FIRST
reduces their validation losses.
Figure 7 further explains this improvement
through phase-wise validation loss for the NIST
Belt task. For the Base Policy + Torque base-
line, loss is higher on pre-contact and con-
tact samples than on free-motion samples, in-
dicating that these phases are harder to model.
By upsampling pre-contact and contact data,
FIRST reduces their validation losses and
brings them closer to the free-motion loss, fo-
cusing training on the phases most critical for
contact-rich manipulation.
FIRST upsampling ablation. We ablate which phases to upsample for best policy performance.
As shown in Table 2, upsampling pre-contact samples achieves the highest average task progress,
Upsampling Phase
Task
C
PC
PC+C
LEGO Assembly
0.697
0.913
0.875
NIST Belt
0.494
0.767
0.649
NIST Insertion
0.800
0.933
0.933
Tool Clean Up
0.650
0.805
0.775
Cap Screwing
0.707
0.673
0.825
Average
0.670
0.818
0.811
Table 2: Upsampling ablation. C: contact,
PC: pre-contact, PC+C: pre-contact + contact.
suggesting that the moments before contact are
especially important as they determine reaching
the correct pose and alignment. Upsampling both
pre-contact and contact phases is also competitive,
and performs best on Cap Screwing, where sus-
tained contact is essential.
In contrast, upsam-
pling only contact samples does not improve perfor-
mance. Overall, these results show that emphasiz-
ing contact-relevant phases, especially pre-contact,
improves long-horizon manipulation.
Additional
results on the highest performing upsampling ratio
for each task are provided in Appendix D.4.
Why not use raw joint current as input? Since low-cost arms often expose motor current, we
compare policies conditioned on raw current with those conditioned on NEXT-estimated external
torque. As shown in Table 10, raw current improves NIST Insertion but not Cap Screwing, while
NEXT-estimated external torque performs best on both tasks. This suggests that subtracting learned
free-space dynamics yields a cleaner contact representation than raw current alone, further high-
lighting the usefulness of NEXT. Full results and attention analysis are in Appendix D.6.
8
Conclusion
We present NEXT, a data-driven method for estimating external joint torque without dedicated force
sensors. With only 10 minutes of free-space data and 1 minute of training, NEXT produces estimates
comparable to dedicated sensor measurements and enables force-feedback teleoperation on low-cost
arms. We further introduce FIRST, which uses NEXT torque estimates to up-sample pre-contact
and contact segments during behavior cloning, improving performance on contact-rich tasks by
emphasizing the phases most critical for alignment and interaction.
8



<!-- ===================== PDF Page 9 ===================== -->

9
Limitations
NEXT has two main limitations. First, its absolute torque scale depends on the motor torque constant
K, which is typically provided by the manufacturer or derived from a current–torque curve. If K
is inaccurate, absolute scale requires calibration against an external force sensor. In practice, this
is less limiting for force-feedback teleoperation, where forces are often scaled for comfort [6], and
also for policy learning, where inputs are normalized anyways. Second, NEXT is robot-specific and
may require retraining across arms due to hardware variation.
Acknowledgments
We thank Andrew Wang, Yulong Li, Koki Yamane, Yangcen Liu, Yongliang Wang, Ritvik Singh,
Zheyuan Hu, Adam Kan, and Arthur Allshire for valuable discussions and feedback on the paper.
We also thank Zheyuan Hu and members of the AIRe Lab for generously providing access to their
YAM arms for this project. This work was supported in part by AFOSR FA9550-23-1-0747, ONR
MURI N00014-22-1-2773 and ONR MURI N00014-24-1-2748.
References
[1] ATI Industrial Automation.
Gamma Force/Torque sensor.
https://www.ati-ia.com/
products/ft/ft_models.aspx?id=gamma, 2026. Accessed: April 28, 2026.
[2] Bota Systems. 6-Axis Force Torque Sensors for Robotics. https://www.botasys.com/
force-torque-sensors, 2024. Accessed: April 28, 2026.
[3] H. Choi, J. E. Low, T. M. Huh, S. Hong, G. A. Uribe, K. A. Hoffmann, J. Di, T. G. Chen, A. A.
Stanley, and M. R. Cutkosky. Coinft: A coin-sized, capacitive 6-axis force torque sensor for
robotic applications. arXiv preprint arXiv:2503.19225, 2025.
[4] R. Bhirangi, T. Hellebrekers, C. Majidi, and A. Gupta. Reskin:versatile, replaceable, lasting
tactile skins. In CoRL, 2021.
[5] M. Y. Cao, S. Laws, and F. R. y Baena. Six-axis force/torque sensors for robotics applications:
A review. IEEE Sensors Journal, 21(24):27238–27251, 2021.
[6] J. J. Liu, Y. Li, K. Shaw, T. Tao, R. Salakhutdinov, and D. Pathak. FACTR: Force-Attending
Curriculum Training for Contact-Rich Policy Learning. In Proceedings of Robotics: Science
and Systems, LosAngeles, CA, USA, June 2025. doi:10.15607/RSS.2025.XXI.079.
[7] S. Oh, T. Takahashi, C. C. Beltran-Hernandez, Y. Kuroda, and M. Hamaya.
A soft wrist
with anisotropic and selectable stiffness for robust robot learning in contact-rich manipulation,
2026. URL https://arxiv.org/abs/2602.14434.
[8] T. Buamanee, M. Kobayashi, Y. Uranishi, and H. Takemura.
Bi-act: Bilateral control-
based imitation learning via action chunking with transformer.
In 2024 IEEE Interna-
tional Conference on Advanced Intelligent Mechatronics (AIM), pages 410–415, 2024. doi:
10.1109/AIM55361.2024.10637173.
[9] K. Yamane, Y. Li, M. Konosu, K. Inami, J. Oaki, T. Tsuji, and S. Sakaino. Design and exper-
imental validation of sensorless 4-channel bilateral teleoperation for low-cost manipulators,
2026. URL https://arxiv.org/abs/2507.06174.
[10] H. Shi, S. Hu, Y. Hou, W. Wang, K. Liu, and S. Song. Minimalist compliance control, 2026.
URL https://arxiv.org/abs/2603.00913.
[11] K. Inami, K. Yamane, and S. Sakaino. Loss function considering dead zone for neural net-
works, 2024. URL https://arxiv.org/abs/2402.00393.
9



<!-- ===================== PDF Page 10 ===================== -->

[12] A. Zhu, Y. Tanaka, F. Rafeedi, and D. Hong. Cycloidal quasi-direct drive actuator designs with
learning-based torque estimation for legged robotics. In 2025 IEEE International Conference
on Robotics and Automation (ICRA), pages 1–8. IEEE, 2025.
[13] K. M. Park, J. Kim, J. Park, and F. C. Park.
Learning-based real-time detection of robot
collisions without joint torque sensors. IEEE Robotics and Automation Letters, 6(1):103–110,
2021. doi:10.1109/LRA.2020.3033269.
[14] J. Liang and O. Kroemer. Contact localization for robot arms in motion without torque sensing.
In 2021 IEEE International Conference on Robotics and Automation (ICRA), pages 6322–
6328. IEEE, 2021.
[15] K. M. Park, Y. Park, S. Yoon, and F. C. Park. Collision detection for robot manipulators using
unsupervised anomaly detection algorithms. IEEE/ASME Transactions on Mechatronics, 27
(5):2841–2851, 2022. doi:10.1109/TMECH.2021.3119057.
[16] N. Yilmaz, J. Y. Wu, P. Kazanzides, and U. Tumerdem. Neural network based inverse dy-
namics identification and external force estimation on the da vinci research kit. In 2020 IEEE
International Conference on Robotics and Automation (ICRA), pages 1387–1393. IEEE, 2020.
[17] Z. Zhang, H. Xu, Z. Yang, C. Yue, Z. Lin, H.-a. Gao, Z. Wang, and H. Zhao. Elucidating
the design space of torque-aware vision-language-action models.
In J. Lim, S. Song, and
H.-W. Park, editors, Proceedings of The 9th Conference on Robot Learning, volume 305 of
Proceedings of Machine Learning Research, pages 4019–4037. PMLR, 27–30 Sep 2025. URL
https://proceedings.mlr.press/v305/zhang25k.html.
[18] T. Kamijo, C. C. Beltran-Hernandez, and M. Hamaya. Learning variable compliance control
from a few demonstrations for bimanual robot with haptic feedback teleoperation system. In
IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), 2024.
[19] J. Yu, H. Liu, Q. Yu, J. Ren, C. Hao, H. Ding, G. Huang, G. Huang, Y. Song, P. Cai, W. Zhang,
and C. Lu. ForceVLA: Enhancing VLA models with a force-aware moe for contact-rich ma-
nipulation. In The Thirty-ninth Annual Conference on Neural Information Processing Systems,
2026. URL https://openreview.net/forum?id=2845H8Ua5D.
[20] Z. He, H. Fang, J. Chen, H.-S. Fang, and C. Lu. Foar: Force-aware reactive policy for contact-
rich robotic manipulation. IEEE Robotics and Automation Letters, 10(6):5625–5632, 2025.
doi:10.1109/LRA.2025.3560871.
[21] W. Liu, J. Wang, Y. Wang, W. Wang, and C. Lu. Forcemimic: Force-centric imitation learning
with force-motion capture system for contact-rich manipulation. In 2025 IEEE International
Conference on Robotics and Automation (ICRA), 2025.
[22] H. Ge, Y. Jia, Z. Li, Y. Li, Z. Chen, R. Huang, and G. Zhou. Filic: Dual-loop force-guided
imitation learning with impedance torque control for contact-rich manipulation tasks, 2025.
URL https://arxiv.org/abs/2509.17053.
[23] H. Xue, J. Ren, W. Chen, G. Zhang, Y. Fang, G. Gu, H. Xu, and C. Lu. Reactive diffusion
policy: Slow-fast visual-tactile policy learning for contact-rich manipulation. In Proceedings
of Robotics: Science and Systems (RSS), 2025.
[24] X. B. Peng, A. Kumar, G. Zhang, and S. Levine. Advantage-weighted regression: Simple and
scalable off-policy reinforcement learning. arXiv preprint arXiv:1910.00177, 2019.
[25] Z. Wang, A. Novikov, K. Zolna, J. T. Springenberg, S. Reed, B. Shahriari, N. Siegel, J. Merel,
C. Gulcehre, N. Heess, and N. de Freitas. Critic regularized regression. Advances in Neural
Information Processing Systems, 33:7768–7778, 2020.
10



<!-- ===================== PDF Page 11 ===================== -->

[26] A. Nair, A. Gupta, M. Dalal, and S. Levine. Awac: Accelerating online reinforcement learning
with offline datasets. arXiv preprint arXiv:2006.09359, 2020.
[27] S. M. Xie, H. Pham, X. Dong, N. Du, H. Liu, Y. Lu, P. S. Liang, Q. V. Le, T. Ma, and A. W.
Yu. Doremi: Optimizing data mixtures speeds up language model pretraining. Advances in
Neural Information Processing Systems, 36:69798–69818, 2023.
[28] J. Hejna, C. A. Bhateja, Y. Jiang, K. Pertsch, and D. Sadigh. Remix: Optimizing data mix-
tures for large scale imitation learning.
In P. Agrawal, O. Kroemer, and W. Burgard, ed-
itors, Proceedings of The 8th Conference on Robot Learning, volume 270 of Proceedings
of Machine Learning Research, pages 145–164. PMLR, 06–09 Nov 2025.
URL https:
//proceedings.mlr.press/v270/hejna25a.html.
[29] A. Ilyas, S. M. Park, L. Engstrom, G. Leclerc, and A. Madry. Datamodels: Understanding
predictions with data and data with predictions. In ICML, 2022.
[30] S. Dass, A. Khaddaj, L. Engstrom, A. Madry, A. Ilyas, and R. Mart´ın-Mart´ın. DataMIL:
Selecting data for robot imitation learning with datamodels. In The Fourteenth International
Conference on Learning Representations, 2026. URL https://openreview.net/forum?
id=AcTsKglDdh.
[31] J. Hejna, S. Mirchandani, A. Balakrishna, A. Xie, A. Wahid, J. Tompson, P. Sanketi, D. Shah,
C. Devin, and D. Sadigh. Robot data curation with mutual information estimators. In Robotics:
Science and Systems (RSS), 2025.
[32] Y. Zhang, Y. Xie, H. Liu, R. Shah, M. Wan, L. Fan, and Y. Zhu. Scizor: Self-supervised data
curation for large-scale imitation learning. In IEEE International Conference on Robotics and
Automation (ICRA), 2026.
[33] Y. J. Ma, J. Hejna, C. Fu, D. Shah, J. Liang, Z. Xu, S. Kirmani, P. Xu, D. Driess, T. Xiao,
et al. Vision language models are in-context value learners. In The Thirteenth International
Conference on Learning Representations, 2024.
[34] Q. Chen, J. Yu, M. Schwager, P. Abbeel, F. Shentu, and P. Wu.
SARM: Stage-aware re-
ward modeling for long horizon robot manipulation. In The Fourteenth International Con-
ference on Learning Representations, 2026. URL https://openreview.net/forum?id=
aemqAxScl9.
[35] A. S. Chen, A. M. Lessing, Y. Liu, and C. Finn. Curating demonstrations using online experi-
ence. In Proceedings of Robotics: Science and Systems, 2025. doi:10.15607/RSS.2025.XXI.
071.
[36] C. Agia, R. Sinha, J. Yang, R. Antonova, M. Pavone, H. Nishimura, M. Itkina, and J. Bohg.
Cupid: Curating data your robot loves with influence functions.
In J. Lim, S. Song, and
H.-W. Park, editors, Proceedings of The 9th Conference on Robot Learning, volume 305 of
Proceedings of Machine Learning Research, pages 2907–2932. PMLR, 27–30 Sep 2025. URL
https://proceedings.mlr.press/v305/agia25a.html.
[37] A. Jubien, M. Gautier, and A. Janot. Dynamic identification of the kuka lwr robot using motor
torques and joint torque sensors data. In 19th IFAC World Congress, 2014.
[38] S. Haddadin, A. De Luca, and A. Albu-Sch¨affer. Robot collisions: A survey on detection,
isolation, and identification. IEEE Transactions on Robotics, 33(6):1292–1312, 2017.
[39] M. Linderoth, A. Stolt, A. Robertsson, and R. Johansson. Robotic force estimation using motor
torques and modeling of low velocity friction disturbances. In 2013 IEEE/RSJ International
Conference on Intelligent Robots and Systems, pages 3550–3556, 2013.
11



<!-- ===================== PDF Page 12 ===================== -->

[40] N. M. Kircanski and A. A. Goldenberg. An experimental study of nonlinear stiffness, hystere-
sis, and friction effects in robot joints with harmonic drives and torque sensors. The Interna-
tional Journal of Robotics Research, 16(2):214–239, 1997.
[41] M. Reuss, N. van Duijkeren, R. Krug, P. Becker, V. Shaj, and G. Neumann. End-to-end learning
of hybrid inverse dynamics models for precise and compliant impedance control. In Robotics:
Science and Systems, 2022.
[42] X. Liu, C. Gong, and qiang liu. Flow straight and fast: Learning to generate and transfer data
with rectified flow. In The Eleventh International Conference on Learning Representations,
2023. URL https://openreview.net/forum?id=XVjTT1nw5z.
[43] S. Mamedov and S. Mikhel. Practical aspects of model-based collision detection. Frontiers in
Robotics and AI, 7:571574, 2020.
[44] P. F. Hokayem and M. W. Spong.
Bilateral teleoperation:
An historical survey.
Au-
tomatica, 42(12):2035–2057, 2006.
ISSN 0005-1098.
doi:https://doi.org/10.1016/j.
automatica.2006.06.027.
URL https://www.sciencedirect.com/science/article/
pii/S0005109806002871.
[45] M. Reuss, ¨O. E. Ya˘gmurlu, F. Wenzel, and R. Lioutikov. Multimodal diffusion transformer:
Learning versatile behavior from multimodal goals. In Proceedings of Robotics: Science and
Systems, Delft, Netherlands, July 2024. doi:10.15607/RSS.2024.XX.121.
[46] T. Z. Zhao, V. Kumar, S. Levine, and C. Finn. Learning Fine-Grained Bimanual Manipulation
with Low-Cost Hardware. In Proceedings of Robotics: Science and Systems, Daegu, Republic
of Korea, July 2023. doi:10.15607/RSS.2023.XIX.016.
[47] C. Gaz, M. Cognetti, A. Oliva, P. Robuffo Giordano, and A. De Luca. Dynamic identifi-
cation of the franka emika panda robot with retrieval of feasible parameters using penalty-
based optimization.
IEEE Robotics and Automation Letters, 4(4):4147–4154, 2019.
doi:
10.1109/LRA.2019.2931248.
[48] AgileX Robotics. agx arm urdf: Agilex robot arm urdf models. https://github.com/
agilexrobotics/agx_arm_urdf, 2026. Accessed: 2026-05-11.
[49] P. Wu, Y. Shentu, Z. Yi, X. Lin, and P. Abbeel. GELLO: A general, low-cost, and intuitive
teleoperation framework for robot manipulators. In 2024 IEEE/RSJ International Conference
on Intelligent Robots and Systems (IROS), pages 12156–12163. IEEE, 2024. doi:10.1109/
IROS58592.2024.10801581.
[50] O. Sim´eoni, H. V. Vo, M. Seitzer, F. Baldassarre, M. Oquab, C. Jose, V. Khalidov,
M. Szafraniec, S. Yi, M. Ramamonjisoa, et al. Dinov3. arXiv preprint arXiv:2508.10104,
2025.
[51] K. Shaw and D. Pathak. Leap hand v2: Dexterous, low-cost anthropomorphic hybrid rigid soft
hand for robot learning. In 2nd Workshop on Dexterous Manipulation: Design, Perception and
Control (RSS).
12



<!-- ===================== PDF Page 13 ===================== -->

Appendix
A
NEXT Implementation Details
A.1
Free-Space Inverse Dynamics Model Architecture
We use an LSTM-based sequence model for free-space inverse dynamics prediction. Rather than
maintaining a persistent hidden state across the entire trajectory, we adopt a stateless sliding-window
formulation. The network therefore implicitly models temporal dynamics within the look-back win-
dow, while the hidden state is reset for each sequence. This formulation provides recent motion
context without accumulating hidden-state drift over long trajectories. The local history allows the
model to capture velocity-dependent, actuator-dependent, and unmodeled friction effects without
explicitly estimating acceleration. For deployment, the estimator can be evaluated using a rolling
buffer of the most recent N observations. External joint torque is then estimated as the residual
between the measured joint torque and the predicted free-space torque. The network architecture
and training hyperparameters are listed in Table 3.
On a single thread of an Intel Core i9-10920X CPU, the free-space inverse dynamics model required
approximately 1.76 ms per forward pass, corresponding to an effective update rate of 568 Hz. This
enables high-frequency force feedback teleoperation. In practice, we run the model at 100 Hz.
Hyperparameter
Value
Recurrent backbone
2-layer LSTM with hidden size 128
Prediction head
2-layer MLP with hidden size 256
Dropout
0.1
Loss function
Mean squared error (MSE)
Optimizer
AdamW
Base learning rate
1 × 10−3
Weight decay
1 × 10−6
Gradient clipping
1.0
Early stopping
Enabled; validation loss, patience 20, warmup 10, min ∆= 10−5
Hardware
NVIDIA RTX 3090, 24 GB
Training time
Approximately 1 minute with early stopping
Data frequency
100Hz
Table 3: Network architecture and training hyperparameters for the free-space inverse dynamics
neural network used in NEXT.
A.2
Free-Space Inverse Dynamics Network Ablations
To select the free-space inverse dynamics model used in NEXT, we ablate model architecture, input
modality, and history length using external torque estimation error in both free-space and contact
settings. Prior work has used MLPs for torque estimation on specialized da Vinci systems [16] and
GRUs for single-actuator external torque estimation [12]. We therefore compare MLP, GRU, and
LSTM architectures.
Table 4 reports the architecture ablation results, with values corresponding to the average joint L1
validation error between predicted free-space torque and ground-truth free-space torque on the held-
out set. MLPs perform worse than recurrent models, showing the importance of temporal history for
free-space torque estimation. GRUs perform well in free space, but performs worse during contact.
LSTMs achieve the best free-space and contact performance, where longer-term actuator, frictional,
and controller-dependent effects may play a larger role.
We further ablate input modalities and history length. Specifically, we compare histories of joint
positions q, joint positions and velocities (q, ˙q), and joint positions, velocities, and tracking error
(q, ˙q, ∆qd), with history lengths of 10, 25, and 50 steps. Using (q, ˙q, ∆qd) consistently outperforms
the other inputs, suggesting that tracking error provides useful information about controller effort
13



<!-- ===================== PDF Page 14 ===================== -->

Input
H
MLP
GRU
LSTM (Ours)
Free Motion ↓
Contact ↓
Free Motion ↓
Contact ↓
Free Motion ↓
Contact ↓
q
10
2.35±2.34
1.78±1.64
2.25±1.79
1.12±1.09
2.53±2.52
2.61±2.29
25
2.02±2.01
1.34±1.32
1.68±1.30
0.665±0.478
2.21±2.20
1.45±1.39
50
1.76±1.76
1.13±1.06
1.62±1.24
0.843±0.835
2.15±2.15
1.09±0.908
q, ˙q
10
1.04±1.03
0.681±0.647 0.863±0.862 0.580±0.535
1.15±1.15
0.715±0.553
25 0.980±0.976 0.908±0.802 0.814±0.814 0.596±0.506
1.11±1.11
0.551±0.511
50
1.02±1.02
0.905±0.892 0.795±0.794 0.653±0.549
1.12±1.12
0.578±0.526
q, ˙q, ∆qd
10 0.706±0.702
1.89±1.71
0.423±0.422
1.94±1.75
0.504±0.503
0.876±0.244
25 0.755±0.747
1.56±1.45
0.473±0.472
1.47±1.29
0.490±0.489
0.789±0.453
50 0.813±0.811
1.47±1.33
0.429±0.429
1.62±1.45
0.414±0.278 0.547±0.348
Table 4: Ablation of model family, input modality, and temporal history length for external torque
estimation. Values corresponding to the average joint L1 validation error between predicted free-
space torque and ground-truth free-space torque on the held-out set, reported in Nm.
and actuator response. Short histories of 10 steps generally yield higher error, while increasing the
history to 50 steps improves performance, especially for LSTM models. Based on these results, we
use an LSTM with history length H = 50 and input (q, ˙q, ∆qd) as the final NEXT free-space inverse
dynamics model.
A.3
Free-Space Data Collection Guidelines for NEXT
Figure 8 shows the distribution of the 10-minute free-motion dataset collected on the Piper arms
for training the free-space inverse dynamics LSTM for NEXT. During data collection, we follow
a simple coverage-oriented procedure: first, each joint is moved independently across its range of
motion; next, the end effector is moved through Cartesian-like motions involving multiple joints;
finally, both slow and fast motions are repeated to capture velocity-dependent dynamics. Although
these trajectories can be generated automatically through motion planning, in practice they can also
be recorded once and replayed on new robot setups to quickly collect training data.
Figure 8: Distribution of the 10-minute free-motion dataset used to train NEXT on the Piper arm.
The plots show the coverage of joint-position error ∆q, joint position q, joint velocity ˙q, and free-
space torque τf across all six joints.
14



<!-- ===================== PDF Page 15 ===================== -->

A.4
External Joint Torque Estimation Baseline Implementation
We implement two model-based external joint torque estimation baselines for NEXT: FILIC [22]
and a disturbance observer [43]. We evaluate both methods on the Franka Panda and AgileX Piper
arms. For the Franka, we use the dynamics model from [47], whose parameters were obtained
through optimization-based system identification. For the AgileX Piper, we use the dynamics model
derived from the manufacturer-provided URDF [48].
FILIC [22]. For FILIC, we use the standard residual-based estimate introduced in Sec. 3. Given
the measured motor torque τm, external torque is estimated by subtracting the nominal free-space
torque computed using MuJoCo inverse dynamics:
ˆτ FILIC
ext
= τm −τf,
τf = M(q)¨q + C(q, ˙q) ˙q + g(q)
(9)
This method directly attributes the residual between measured motor torque and modeled free-space
dynamics to external interaction.
Disturbance Observer [43]. As a second baseline, we implement a momentum-based disturbance
observer. Instead of directly computing acceleration-dependent inverse dynamics, the observer esti-
mates external torque from the residual in generalized momentum,
p = M(q) ˙q.
(10)
It maintains an internal momentum estimate ˆp with dynamics
˙ˆp = τm −C(q, ˙q) ˙q −g(q) + ˙M(q) ˙q + ˆτ MDO
ext
,
ˆτ MDO
ext
= Ko(p −ˆp).
(11)
By avoiding direct computation of ¨q, the momentum observer is less sensitive to encoder noise
and produces smoother estimates than FILIC. As shown in Figure 5, it is less noisy than FILIC but
remains less accurate than NEXT.
A.5
Force-Feedback Teleoperation Baseline Implementation
For the force-feedback teleoperation user study, we implement five feedback conditions on the
Franka Panda:
• No Feedback: A GELLO leader-follower system [49] without force feedback.
• Disturbance Observer (DO): External joint torque is estimated on the follower arm using the
disturbance observer described in Appendix A.4, then relayed to the leader arm as force feedback.
• Position-Position Feedback (PP): Feedback torque is computed by scaling the joint position dif-
ference between the leader and follower arms, following [44].
• FACTR Teleop: The FACTR teleoperation system [6], which feeds back external joint torque
from dedicated joint torque sensors, along with gravity compensation, friction compensation, and
null-space regulation.
• Ours: The same teleoperation framework as FACTR Teleop, but using NEXT-predicted external
joint torque instead of dedicated joint torque sensor measurements.
We repeat the same study on the AgileX Piper arm, excluding FACTR Teleop because the Piper does
not provide dedicated joint torque sensing.
A.6
AgileX Piper NEXT Evaluations
Since the AgileX Piper does not provide dedicated external joint torque sensing, we evaluate torque
estimation only in the free-space setting, where the ground-truth external torque is known to be zero.
As shown in Table 5, NEXT achieves the lowest error, with an average L1 error of only 0.018 Nm,
outperforming both FILIC and the disturbance observer. These results show that NEXT can produce
accurate external torque estimates even on a low-cost arm without dedicated force sensors.
To evaluate NEXT in contact settings, we perform a force-feedback teleoperation user study. As
shown in Figure 9, participants rated teleoperation with NEXT-based force feedback as easier to use
than the baseline feedback methods.
15



<!-- ===================== PDF Page 16 ===================== -->

External Torque Method
Error (Nm) ↓
Free Motion (GT)
0.000 ± 0.000
FILIC
0.435 ± 0.225
Disturbance Observer
0.248 ± 0.132
Ours
0.018 ± 0.012
Table 5: Average L1 joint torque error on the Ag-
ileX Piper in free-space settings. Free-space er-
rors are measured against zero external torque.
NEXT achieves the lowest error.
Figure 9: User study results comparing exter-
nal torque feedback methods on the Piper.
B
Behavior Cloning Policy Architecture and Training Hyperparameters
Our behavior cloning policy takes RGB images, proprioceptive inputs, and force estimates as input.
Each modality is tokenized before being passed to either an ACT or Flow Matching DiT policy
architecture. Images are encoded with a DINOv3-Base encoder [50], while proprioceptive and force
inputs are projected into tokens using MLP layers.
Proprioceptive inputs include joint positions and gripper widths, and force inputs correspond to es-
timated external joint torques. We use only the most recent observation at the current timestep. The
action space consists of absolute joint angles and gripper width, with an action prediction horizon
of 30. We note that the action head of each policy is randomly initialized and trained from scratch.
Training details for both ACT and Flow Matching policies are provided in Table 6.
C
Behavior Cloning Evaluation Tasks
C.1
Task Description
We evaluate FIRST along with baselines on five long-horizon, contact-rich manipulation tasks:
• LEGO assembly (LEAP Hand V2 [51]): bimanual hand assembly of two small LEGO blocks onto
a bigger LEGO block.
• NIST belt assembly (gripper): bimanual gripper manipulation of two belts around two pulley sets.
• NIST insertion (gripper): insertion of two pulley sets onto pegs on a NIST board.
• Tool Clean Up (gripper): screw driver insertion followed by case assembly.
• Cap screwing (gripper): cap pickup, alignment, and screwing onto a container until tightened.
C.2
Task Progress Rate Calculation
We report task progress rate in addition to binary success rate to evaluate partial completion in multi-
stage manipulation tasks. Each task is divided into several stages, and each stage is marked as either
completed (s) or failed (f) for every rollout. The task progress rate is calculated as the percentage
of completed stages across all rollouts.
The task stages are listed in Table 7. For example, the LEGO task consists of grasping, aligning,
and inserting both the red and green cubes. The NIST Belt task includes picking up and aligning
both belt components before pushing them into place. The NIST Insertion task measures progress
through picking, aligning, and inserting the black part, followed by picking up the aluminum part.
The Case Assembly task evaluates whether the robot can pick up, align, and insert the driver, then
assemble the box.
For the cap screwing task, we use a modified progress score because a policy can fail by continuing
to rotate the cap after it is already tightened. Therefore, we subtract a penalty based on the number of
over-turns. Each over-turn reduces the progress score by 0.02. This penalizes policies that complete
the screwing motion but do not stop properly.
16



<!-- ===================== PDF Page 17 ===================== -->

Hyperparameter
Value
Behavior Policy Training
Optimizer
AdamW
Base Learning Rate
3 × 10−4
Weight Decay
0.05
Optimizer Momentum
β1, β2 = 0.9, 0.95
Batch Size
128
Learning Rate Schedule
Cosine Decay
Total Epochs
15–20
Warmup Steps
500
Augmentation
RandomResizeCrop
GPU
A6000 (48 GB)
Wall-Clock Time
12–24 hours
Action Chunking Transformer Details
# Encoder Layers
4
# Decoder Layers
6
# MHSA Heads
8
Hidden Dim
768
Feed-Forward Dim
3072
Dropout
0.1
Positional Encoding
Sinusoidal
Action Chunk
30
Flow Matching Details
# Decoder Layers
6
# MHSA Heads
8
Hidden Dim
768
Feed-Forward Dim
3072
Dropout
0.1
Positional Encoding
Sinusoidal
Action Chunk
30
Eval Steps
10
Time Sampling
τ ∼Beta(1.0, 1.5)
ODE Solver
Euler
Noise Distribution
N(0, I)
Table 6: Policy Architecture and Training Hyperparameters
C.3
Imitation Learning with Force Information Baseline Implementations
Base Policy: We use the base ACT or flow-matching policy described in Sec. 6.2. The policy takes
multi-view RGB images and robot proprioception, including joint positions and gripper states, as
input and predicts a chunk of future joint and gripper commands.
Base Policy + Torque: This baseline augments the base policy with external joint torque predictions
from NEXT. The predicted torque signal is embedded with an MLP and added to the policy context
as an additional observation token.
FACTR [6]: FACTR keeps the policy architecture unchanged and applies a visual curriculum to
all image observations. Following the original FACTR implementation, we use a Gaussian blur
operator with a linear curriculum schedule. Specifically, at training step n, the blur strength is set as
σn = σ0

1 −n
N

,
17



<!-- ===================== PDF Page 18 ===================== -->

Task
Stages
LEGO
red grasp, red align, red insert, green grasp, green align, green insert
NIST Belt
pick orange, align big, align small, push, pick grey, align grey, push
NIST Insertion
pick black, align black, insert black, pick aluminum
Tool Clean Up
pick driver, align driver, insert driver, assemble box
Cap Screwing
pick cap, align cap, screw on, end properly
Table 7: Stage decomposition used for task progress calculation.
where σ0 is the initial blur strength and N is the total number of curriculum steps. Thus, the input
images are strongly blurred at the beginning of training, and the blur is gradually removed as training
progresses.
TA-VLA[17]: TA-VLA uses the predicted torque/contact signal as an auxiliary learning target in
addition to the action. During inference, only the robot-action dimensions are executed, while the
auxiliary torque/contact prediction is used only to shape the learned representation during training.
D
Additional Results on FIRST
D.1
Segmenting Trajectories into Contact Phases Visualization
Figure 10 shows an example of phase labels produced by our hysteresis-threshold-based labeling
procedure. The estimated external torque from NEXT remains near zero during free motion and
rises clearly during physical interaction, allowing the trajectory to be separated into free-motion,
pre-contact, and contact phases. The pre-contact label captures the short interval immediately be-
fore contact onset, where precise alignment is required, while the contact label captures periods of
sustained interaction. This sensor-level signal enables automatic segmentation of demonstrations
even on robot arms without dedicated force sensors, providing the phase annotations used by FIRST
for force-informed re-sampling.
Figure 10: Contact phase segmentation from learned external torque. The normalized τext from
NEXT is used to segment demonstrations into free motion, pre-contact, and contact phases. The
image overlays show that the predicted labels align with the robot’s interaction state during the task.
18



<!-- ===================== PDF Page 19 ===================== -->

D.2
Task Progress Evaluation on ACT
We also apply FIRST to ACT, in addition to the flow-matching policy presented in Section 7. As
shown in Figure 11, we observe a similar trend: upsampling pre-contact and contact phases improves
task progress over the baselines.
LEGO Assembly
NIST Belt
NIST Insertion
Tool Clean Up
Cap Screwing
0%
20%
40%
60%
80%
100%
Task Progress
Base Policy
Base Policy + Torque
FACTR
TA-VLA
FIRST
Figure 11: Task progress across five contact-rich manipulation tasks using ACT policy.
D.3
Selected Re-Sampling Ratios for Each Task
We report the task-specific re-sampling ratios used by FIRST in Table 8. All ratios are written as
wF : wPC : wC, where wF, wPC, and wC denote the sampling weights for free-motion, pre-contact,
and contact samples, respectively. For most tasks, FIRST uses 1 : 5 : 1, which up-samples pre-
contact samples by 5× relative to free-motion and contact samples. For Cap Screwing, FIRST uses
1 : 3 : 3, up-sampling both pre-contact and contact samples, since sustained contact is central to
successful execution. These ratios are selected based on task progress for each task.
Task
Best Re-Sampling Ratio
LEGO Assembly
1 : 5 : 1
NIST Belt Assembly
1 : 5 : 1
NIST Insertion
1 : 5 : 1
Tool Clean Up
1 : 5 : 1
Cap Screwing
1 : 3 : 3
Table 8: Best re-sampling ratios used by FIRST for each task. Ratios are reported as wF : wPC : wC,
corresponding to free-motion, pre-contact, and contact sampling weights.
D.4
Up-Sampling Magnitude Ablation
Figure 12 ablates the effect of up-sampling magnitude on task progress. We define the magnitude
m as the sampling weight assigned to the up-sampled phase relative to free motion. For pre-contact
up-sampling, the phase weights are
wF : wPC : wC = 1 : m : 1,
whereas for pre-contact + contact up-sampling, the phase weights are
wF : wPC : wC = 1 : m : m.
We run this ablation on NIST Belt Assembly, NIST Insertion, and Tool Clean Up using ACT. Due to
limited time and compute, we additionally run the flow-matching ablation only on Tool Clean Up.
Across tasks, moderate up-sampling improves task progress, with performance generally peaking
around m = 5. Larger values lead to saturation or degradation, suggesting that overly aggressive
up-sampling can over-bias the training distribution toward contact-relevant segments and reduce
coverage of the full task trajectory.
D.5
Up-Sampling Policies Without Force Input
While Section 7 evaluates FIRST on policies that condition on external joint torque, FIRST can
also be applied without using force as a policy input. In this setting, NEXT is used only offline
19



<!-- ===================== PDF Page 20 ===================== -->

Figure 12: Moderate up-sampling generally improves performance, with best results around m = 5,
while larger values lead to saturation or degradation.
to segment demonstrations into free-motion, pre-contact, and contact phases. These phase labels
are then used by FIRST to re-sample the training data, increasing the frequency of pre-contact and
contact examples in each minibatch. Importantly, the policy itself receives only the standard visual
and proprioceptive observations, without the predicted external joint torque.
Table 9 highlights two trends. First, when comparing methods within the columns without external
joint torque, phase-based upsampling improves over standard behavior cloning. For both ACT and
Flow Matching, using PC or PC+C upsampling generally increases task success compared to no
upsampling, showing that NEXT-derived phase labels are useful even when force estimates are used
only at training time for data re-sampling. This indicates that FIRST can improve standard behavior
cloning by reshaping the training distribution toward task-critical segments where alignment and
contact-sensitive corrections are most important.
Second, when comparing horizontally between policies trained with and without external joint
torque, the force-conditioned policies consistently achieve higher success rates. This shows that
while phase-based upsampling alone provides a clear benefit, directly feeding the predicted external
joint torque into the policy provides additional state information that further improves contact-rich
manipulation. Overall, the best performance is achieved when both components are used together:
NEXT provides external joint torque as a policy input, while FIRST uses NEXT-derived phase labels
to up-sample the most relevant pre-contact and contact phases during training.
Policy
Up-sampling
Strategy
w/o τext
w/ τext
Belt
Assembly
Tool
Clean Up
Belt
Assembly
Tool
Clean Up
ACT
None
0.220
0.600
0.347
0.650
PC
0.360
0.600
0.687
0.650
PC + C
0.283
0.763
0.770
0.788
Flow Matching
None
0.150
0.463
0.431
0.525
PC
0.213
0.575
0.767
0.805
PC + C
0.241
0.525
0.649
0.775
Table 9: Effect of phase-based upsampling on policies trained with and without external joint torque.
Results are reported as task success rates.
D.6
Inputting Joint Current vs. External Joint Torque on Policy Performance
We further examine whether raw joint current is sufficient for contact-aware policy learning, or
whether the learned external joint torque estimate provides a more useful representation. Prior work
such as TA-VLA [17] uses readily available joint current measurements from low-cost robot arms
20



<!-- ===================== PDF Page 21 ===================== -->

Method
NIST Insertion ↑
Cap Screwing ↑
ACT
0.220
0.440
ACT w/ current
0.431
0.440
ACT w/ τext
0.567
0.525
Table 10: Policy performance when conditioning on raw joint current versus learned external joint
torque. Learned external joint torque provides the best performance on both contact-sensitive tasks.
Higher is better. Bold indicates the best result within each task.
as a force-related input for imitation learning. However, joint current is only an indirect measure
of physical interaction: it contains actuator effort required for both free-space motion and external
contact, as well as hardware- and controller-dependent effects. As a result, the policy must implicitly
disentangle contact-induced torque from free-space dynamics.
In contrast, NEXT explicitly estimates external joint torque by subtracting a learned free-space
torque prediction from the measured motor torque. This distinction is important because estimating
a clean external torque signal from current alone generally requires temporal context. As shown in
our NEXT free-space inverse dynamics ablations in Appendix A.2, longer proprioceptive histories
improve torque estimation, suggesting that instantaneous current and state observations are insuf-
ficient to reliably separate free-space actuation from contact. A standard behavior cloning policy,
however, typically receives only the current observation rather than a long proprioceptive history
such as the 50-step history used by NEXT. While one could provide such a history directly to the
policy, this increases the burden on the policy to simultaneously infer dynamics, localize contact,
and solve the visuomotor task. In practice, long proprioceptive histories may also encourage over-
fitting to trajectory-specific motion patterns rather than using vision appropriately to react to scene
variation. NEXT avoids this issue by using history in a specialized force-estimation module, while
providing the policy with a compact, contact-relevant external torque signal.
We compare policies conditioned on raw joint current and learned external joint torque on two
contact-sensitive tasks: NIST Insertion and Cap Screwing. Both tasks require force cues for suc-
cessful execution: NIST Insertion depends on detecting alignment and contact during insertion,
while Cap Screwing requires identifying when the cap has been fully tightened. As shown in Ta-
ble 10, conditioning on joint current improves performance on NIST Insertion compared to the base
ACT policy, but does not improve Cap Screwing. In contrast, conditioning on learned external joint
torque achieves the best performance on both tasks. Qualitatively, we observe that policies condi-
tioned on raw joint current often continue rotating the cap after it is already tightened, leading to
over-screwing failures. Policies conditioned on learned external torque are better able to detect the
tightened contact state and terminate the screwing motion appropriately.
The attention analysis in Figure 13 further supports this observation. To visualize how the pol-
icy uses force-related information during rollout, we compute the average cross-attention from the
action tokens to the observation tokens in the first decoder layer. The decoder attends over both
visual tokens and force-related tokens, but in Figure 13 we plot only the attention mass assigned
to the force-related tokens: raw joint-current tokens for the current-conditioned policy, and learned
external-torque tokens for our policy. This visualization therefore measures how strongly the policy
attends to the force input when predicting actions.
During screw-cap manipulation, the policy conditioned on learned external torque shows a clear
increase in attention around key contact events, especially when the cap becomes tightened and the
screwing motion should stop. In contrast, the policy conditioned on raw joint current exhibits weaker
and less event-aligned attention to its current tokens. While attention is only a diagnostic and does
not by itself prove causality, this pattern suggests that learned external joint torque provides a cleaner
and more task-relevant contact representation than raw joint current for force-sensitive manipulation.
21



<!-- ===================== PDF Page 22 ===================== -->

Figure 13: Attention values during screw-cap manipulation. Learned external torque produces
clearer attention around key contact events than raw joint effort.
D.7
Per-batch Up-sampling vs. Weighted Regression
We further compare FIRST against a weighted-regression baseline, where the loss is reweighted
according to whether each sample belongs to the free-motion, pre-contact, or contact phase. Both
approaches aim to emphasize contact-relevant data, but they do so in different ways: weighted
regression modifies the optimization objective, whereas FIRST changes the training distribution by
increasing the frequency of informative samples within each batch.
As shown in Table 11, weighted regression either degrades performance or provides only limited im-
provement over the no-upsampling baseline. In contrast, weighted sampling consistently improves
LEGO assembly performance. We observe similar qualitative trends in other contact-rich tasks.
This suggests that directly increasing the loss contribution of sparse contact-related samples can
make optimization less stable, potentially causing the policy to overfit to high-weighted segments
or ignore the broader trajectory context. By contrast, per-batch up-sampling preserves the original
regression objective while exposing the policy more frequently to critical pre-contact and contact
transitions, leading to more stable policy learning.
Phase Emphasis Mechanism
Emphasized Phase
Task Progress
None
–
0.453
Weighted Regression
PC
0.345
PC + C
0.478
FIRST (Ours)
PC
0.573
PC + C
0.501
Table 11: Comparison of phase emphasis mechanisms for LEGO assembly using ACT.
22
