<h1>🚦 Smart Traffic Junction (4-Way) — Siemens TIA (LAD + SCL/ST)</h1>

<p>
A 4-way <strong>smart traffic controller</strong> for Siemens S7-1200/1500 built with a minimal <strong>Ladder (LAD)</strong> main and most logic in <strong>SCL/Structured Text (ST)</strong>. 
It supports <em>adaptive green extensions</em> based on vehicle detectors, pedestrian crossing phases with all-red buffers, and tunable timing parameters via a global DB.
</p>

<p>
<b>Blocks:</b> <code>FC "Calc_Extension"</code>, <code>FB "JunctionCtrl"</code>, <code>DB "DB_Params_Junction"</code>, and <code>OB1</code> (LAD call). 
<span class="pill">S7-1200/1500</span> <span class="pill">Optimized access</span>
</p>

<hr />

<h2 id="toc">Table of Contents</h2>
<ol class="toc">
  <li><a href="#features">Features</a></li>
  <li><a href="#architecture">Architecture & Data Flow</a></li>
  <li><a href="#blocks">Blocks & Roles (FC/FB/DB/OB1)</a></li>
  <li><a href="#io">I/O Mapping (example)</a></li>
  <li><a href="#timings">Timing Parameters & Tuning</a></li>
  <li><a href="#state">State Machine & Sequence</a></li>
  <li><a href="#alg">Adaptive Extension Algorithm</a></li>
  <li><a href="#install">Project Setup (TIA) & Build Order</a></li>
  <li><a href="#simulate">Simulation with PLCSIM</a></li>
  <li><a href="#hmi">HMI/Diagnostics Tags</a></li>
  <li><a href="#safety">Safety & Failsafe Behavior</a></li>
  <li><a href="#troubleshoot">Troubleshooting</a></li>
</ol>

<hr />

<h2 id="features">1) Features</h2>
<ul>
  <li><b>4-way junction</b> with <b>N-S</b> and <b>E-W</b> phases.</li>
  <li><b>Adaptive green</b> time: extends green in steps while a queue is present, capped by a per-direction maximum.</li>
  <li><b>Pedestrian requests</b> per roadway with <b>all-red</b> vehicle hold during <i>Walk</i> and <i>Clear</i> intervals.</li>
  <li><b>All-red</b> interlocks during transitions.</li>
  <li>Minimal <b>LAD</b>: OB1 just calls the FB and wires IO/parameters.</li>
  <li>Tunable times via a <b>Global DB</b> (on-line editable).</li>
</ul>

<hr />

<h2 id="architecture">2) Architecture & Data Flow</h2>
<pre><code>┌────────────────────────────────────────────────────────────────────────────┐
│ OB1 (LAD)                                                                  │
│   └─ Calls FB "JunctionCtrl" (Instance DB)                                  │
│        • Wires physical inputs (detectors / ped buttons)                    │
│        • Wires physical outputs (lamp heads / ped walk)                     │
│        • Passes timing parameters from DB "DB_Params_Junction"              │
└────────────────────────────────────────────────────────────────────────────┘
             │
             ▼
┌────────────────────────────────────────────────────────────────────────────┐
│ FB "JunctionCtrl" (SCL)                                                     │
│   • Finite state machine (NS/EW Green & Yellow, All-Red, Ped Walk/Clear)    │
│   • Calls FC "Calc_Extension" to compute green extension per scan/cycle     │
│   • Owns TON timer instances (tGreen, tYellow, tAllRed, tPed)               │
│   • Latches pedestrian requests; sets lamp outputs                          │
└────────────────────────────────────────────────────────────────────────────┘
             │
             ▼
┌────────────────────────────────────────────────────────────────────────────┐
│ FC "Calc_Extension" (SCL)                                                   │
│   • Returns TIME = min(remaining allowance, step) when queue detected       │
│   • Uses TIA SCL variable style with # prefix                               │
└────────────────────────────────────────────────────────────────────────────┘

┌────────────────────────────────────────────────────────────────────────────┐
│ DB "DB_Params_Junction"                                                     │
│   • Min/Max/Step/Yellow/All-Red/Ped Walk/Ped Clear times                    │
│   • Safe to edit online for tuning                                          │
└────────────────────────────────────────────────────────────────────────────┘
</code></pre>

<hr />

<h2 id="blocks">3) Blocks & Roles</h2>

<h3>FC — <code>FC "Calc_Extension"</code> (SCL)</h3>
<p>
Pure function that computes the per-scan green extension for the <i>current</i> green approach.
It returns a <code>TIME</code> value via the function name (TIA SCL style).
</p>

<pre><code>// Signature (all variable references use #)
FUNCTION "Calc_Extension" : TIME
{ S7_Optimized_Access := 'TRUE' }
VAR_INPUT
    HasQueue     : BOOL;
    ElapsedGreen : TIME;
    MaxExt       : TIME;
    Step         : TIME;
END_VAR
VAR_TEMP
    remaining : TIME;
    effStep   : TIME;
END_VAR

#effStep := #Step;
IF #effStep &lt; T#0S THEN
    #effStep := T#0S;
END_IF;

#remaining := #MaxExt - #ElapsedGreen;
IF #remaining &lt; T#0S THEN
    #remaining := T#0S;
END_IF;

IF #HasQueue THEN
    IF #remaining &gt; #effStep THEN
        #Calc_Extension := #effStep;
    ELSE
        #Calc_Extension := #remaining;
    END_IF;
ELSE
    #Calc_Extension := T#0S;
END_IF;

END_FUNCTION
</code></pre>

<h3>FB — <code>FB "JunctionCtrl"</code> (SCL)</h3>
<p>
Implements the <b>finite state machine</b>, owns the IEC <code>TON</code> instances, and drives the outputs. 
It calls <code>Calc_Extension</code> during the active green state to extend the phase in discrete steps until the cap is reached.
Pedestrian requests are latched and inserted as all-red phases between vehicle cycles.
</p>

<details>
  <summary>Key elements inside the FB</summary>
  <ul>
    <li><b>Inputs:</b> <code>Enable</code>, detectors <code>NS_Det/EW_Det</code>, pedestrian requests <code>PedReq_NS/EW</code>, timing params.</li>
    <li><b>Outputs:</b> vehicle lamps <code>NS_R/Y/G</code>, <code>EW_R/Y/G</code>, pedestrian indications <code>PedWalk_NS/EW</code>, <code>CurrState</code> for HMI.</li>
    <li><b>Timers:</b> <code>#tGreen</code>, <code>#tYellow</code>, <code>#tAllRed</code>, <code>#tPed</code> (all <code>TON</code> instances).</li>
    <li><b>States (constants):</b> <code>ST_NS_GREEN</code>, <code>ST_NS_YELLOW</code>, <code>ST_PED_NS_WALK</code>, <code>ST_PED_NS_CLEAR</code>, <code>ST_ALL_RED</code>, <code>ST_EW_GREEN</code>, <code>ST_EW_YELLOW</code>, <code>ST_PED_EW_WALK</code>, <code>ST_PED_EW_CLEAR</code>.</li>
    <li><b>Failsafe:</b> when <code>#Enable = FALSE</code>, outputs force to all-red and pedestrians off.</li>
  </ul>
</details>

<p><b>Example call to the FC inside the FB (TIA style with # vars):</b></p>
<pre><code>#ext := Calc_Extension(
          HasQueue     := #NS_Det,
          ElapsedGreen := #tGreen.ET,
          MaxExt       := #T_MaxExt_NS,
          Step         := #T_ExtStep);

#tGreen(IN := TRUE, PT := #T_MinGreen_NS + #ext);
</code></pre>

<h3>DB — <code>DB "DB_Params_Junction"</code> (Global)</h3>
<p>
Holds tunable timing parameters (all <code>TIME</code>). Recommended defaults:
</p>
<table>
  <thead><tr><th>Tag</th><th>Type</th><th>Default</th><th>Description</th></tr></thead>
  <tbody>
    <tr><td>MinGreen_NS</td><td>TIME</td><td>T#10S</td><td>Base minimum green for N-S</td></tr>
    <tr><td>MinGreen_EW</td><td>TIME</td><td>T#10S</td><td>Base minimum green for E-W</td></tr>
    <tr><td>MaxExt_NS</td><td>TIME</td><td>T#15S</td><td>Max extra beyond MinGreen on N-S</td></tr>
    <tr><td>MaxExt_EW</td><td>TIME</td><td>T#15S</td><td>Max extra beyond MinGreen on E-W</td></tr>
    <tr><td>ExtStep</td><td>TIME</td><td>T#3S</td><td>Extension granularity per evaluation</td></tr>
    <tr><td>Yellow</td><td>TIME</td><td>T#3S</td><td>Amber interval</td></tr>
    <tr><td>AllRed</td><td>TIME</td><td>T#1S</td><td>All-red interlock between phases</td></tr>
    <tr><td>PedWalk</td><td>TIME</td><td>T#6S</td><td>Pedestrian steady walk interval</td></tr>
    <tr><td>PedClear</td><td>TIME</td><td>T#4S</td><td>Pedestrian clearance interval</td></tr>
  </tbody>
</table>

<h3>OB1 — (LAD)</h3>
<p>
OB1 contains a single ladder network that <b>instantiates</b> <code>FB "JunctionCtrl"</code>, wires physical I/O, and passes parameters from the global DB.
</p>

<hr />

<h2 id="io">4) I/O Mapping (example)</h2>

<table>
  <thead><tr><th>Signal</th><th>Address</th><th>Meaning</th></tr></thead>
  <tbody>
    <tr><td>NS_Det</td><td>I0.0</td><td>N-S vehicle detector</td></tr>
    <tr><td>EW_Det</td><td>I0.1</td><td>E-W vehicle detector</td></tr>
    <tr><td>PedReq_NS</td><td>I0.2</td><td>Pedestrian pushbutton to cross N-S roadway</td></tr>
    <tr><td>PedReq_EW</td><td>I0.3</td><td>Pedestrian pushbutton to cross E-W roadway</td></tr>
    <tr><td>NS_R / NS_Y / NS_G</td><td>Q0.0 / Q0.1 / Q0.2</td><td>N-S red, yellow, green lamp heads</td></tr>
    <tr><td>EW_R / EW_Y / EW_G</td><td>Q0.3 / Q0.4 / Q0.5</td><td>E-W red, yellow, green lamp heads</td></tr>
    <tr><td>PedWalk_NS</td><td>Q0.6</td><td>Walk signal across N-S roadway</td></tr>
    <tr><td>PedWalk_EW</td><td>Q0.7</td><td>Walk signal across E-W roadway</td></tr>
  </tbody>
</table>

<p><b>Note:</b> Adjust addresses to your hardware. Keep the symbolic names the same as in the FB call for a quick drop-in.</p>

<hr />

<h2 id="timings">5) Timing Parameters & Tuning</h2>
<ul>
  <li><b>MinGreen_*</b>: guaranteed green time for each approach.</li>
  <li><b>MaxExt_*</b>: total additional time the green may be extended while a queue persists.</li>
  <li><b>ExtStep</b>: the increment applied per evaluation (per scan or per control cycle inside the state).</li>
  <li><b>Yellow</b>: amber time before stopping traffic on the active approach.</li>
  <li><b>AllRed</b>: safety all-red buffer when changing approaches or before pedestrian phases.</li>
  <li><b>PedWalk</b> / <b>PedClear</b>: pedestrian walk and clearance intervals while all vehicles are red.</li>
</ul>

<hr />

<h2 id="state">6) State Machine & Sequence</h2>

<ol>
  <li><b>ST_NS_GREEN</b>: N-S green. Uses <code>Calc_Extension</code> to extend beyond <code>#T_MinGreen_NS</code> up to <code>#T_MaxExt_NS</code> while <code>#NS_Det</code> is true.</li>
  <li><b>ST_NS_YELLOW</b>: N-S yellow. When finished, if <code>#PedReq_NS_L</code> is latched, go to pedestrian walk; otherwise all-red.</li>
  <li><b>ST_PED_NS_WALK</b> → <b>ST_PED_NS_CLEAR</b>: All vehicle signals red. <code>#PedWalk_NS</code> asserted for <code>#T_PedWalk</code>, then clearance for <code>#T_PedClear</code>. Latch clears.</li>
  <li><b>ST_ALL_RED</b>: All-red buffer. Then switch to next vehicle direction.</li>
  <li><b>ST_EW_GREEN</b>: Same as N-S but for E-W with its own Min/MaxExt and detector.</li>
  <li><b>ST_EW_YELLOW</b> → <b>ST_PED_EW_WALK</b> → <b>ST_PED_EW_CLEAR</b> → <b>ST_ALL_RED</b>.</li>
</ol>

<p>
<b>Failsafe:</b> If <code>#Enable = FALSE</code> at any time, outputs force to all-red and pedestrian signals off.
</p>

<hr />

<h2 id="alg">7) Adaptive Extension Algorithm</h2>

<p>The FC implements:</p>
<ul>
  <li>Compute <code>#remaining := max(0, #MaxExt - #ElapsedGreen)</code>.</li>
  <li>Clamp negative <code>#Step</code> to 0 (<code>#effStep</code>).</li>
  <li>If the detector is on (<code>#HasQueue = TRUE</code>), return <code>min(#remaining, #effStep)</code>. Otherwise return <code>T#0S</code>.</li>
</ul>

<p>
In the FB, the green timer <code>#tGreen</code> is started with <code>#PT := #T_MinGreen_* + #ext</code>, where <code>#ext</code> is the FC return value.
This allows multiple small increments across the green phase until the maximum extension is exhausted or the detector drops.
</p>

<hr />

<h2 id="install">8) Project Setup (TIA) & Build Order</h2>

<ol>
  <li>Create a new TIA Portal project targeting S7-1200/1500 (Optimized block access).</li>
  <li>Add <b>FC</b> <code>"Calc_Extension"</code> (Language: SCL) and paste the function code shown above.</li>
  <li>Add <b>FB</b> <code>"JunctionCtrl"</code> (Language: SCL) and paste the state machine code (using <code>#</code> prefixes).</li>
  <li>Create <b>Global DB</b> <code>"DB_Params_Junction"</code> with the timing tags and defaults listed in this README.</li>
  <li>Open <b>OB1</b> and create a single <b>LAD</b> network that calls <code>FB "JunctionCtrl"</code>. When dropping it the first time, TIA will create an <b>Instance DB</b> (e.g., <code>DB_JunctionCtrl</code>).</li>
  <li>Wire the inputs (I0.x), outputs (Q0.x), and parameter fields from <code>"DB_Params_Junction"</code>.</li>
  <li><b>Compile order:</b> FC → FB → OB1. Download to PLC or PLCSIM.</li>
</ol>

<hr />

<h2 id="simulate">9) Simulation with PLCSIM</h2>
<ol>
  <li>Start <b>PLCSIM</b> and load the compiled program.</li>
  <li>Toggle <code>I0.0</code> (<b>NS_Det</b>) and <code>I0.1</code> (<b>EW_Det</b>) to simulate vehicle presence. Observe adaptive green via <code>Q0.2</code> (NS_G) and <code>Q0.5</code> (EW_G).</li>
  <li>Press <code>I0.2</code> or <code>I0.3</code> to request pedestrian crossing. After the current yellow and all-red, Walk signals <code>Q0.6</code>/<code>Q0.7</code> will activate, followed by clearance, then next vehicle phase.</li>
  <li>Adjust timings online in <code>DB_Params_Junction</code> to observe behavior changes live.</li>
</ol>

<hr />

<h2 id="hmi">10) HMI/Diagnostics Tags</h2>
<ul>
  <li><b>Display:</b> <code>CurrState</code> as numeric or via text list mapping to state names.</li>
  <li><b>Indicators:</b> Lamp outputs and Walk outputs for a synoptic mimic.</li>
  <li><b>Controls:</b> Optional HMI toggles for <code>Enable</code>, and a maintenance screen to edit <code>DB_Params_Junction</code> timers.</li>
</ul>

<hr />

<h2 id="safety">11) Safety & Failsafe Behavior</h2>
<ul>
  <li>On <code>#Enable = FALSE</code> the system forces all vehicle signals to <b>red</b> and disables pedestrian signals (<span class="ok">safe</span>).</li>
  <li><b>All-Red</b> buffer (<code>#T_AllRed</code>) separates all transitions, including before and after ped phases.</li>
  <li>Only one approach gets green at a time; pedestrian walk occurs only during all-red vehicle states.</li>
  <li>Consider external hardware interlocks (E-Stop, lamp drive monitoring) for real deployments.</li>
</ul>

<hr />

<h2 id="troubleshoot">12) Troubleshooting</h2>
<ul>
  <li><b>“Unknown identifier” / compile errors:</b> Ensure you used <b>#</b> prefixes for variables inside SCL and that the FC/FB signatures match the code here.</li>
  <li><b>RET_VAL confusion:</b> In SCL, return value is written via <code>#Calc_Extension</code>. In LAD/FBD, the function’s output pin shows as <code>RET_VAL</code>.</li>
  <li><b>Type mismatch (TIME):</b> Ensure all timing parameters and <code>#ext</code> are <code>TIME</code>. <code>#tGreen.ET</code> is <code>TIME</code> on S7-1200/1500.</li>
  <li><b>Wrong phase order:</b> Check <code>#NextIsEW</code> logic and pedestrian latch clearing (<code>#PedReq_*_L := FALSE</code> after clearance).</li>
  <li><b>No extension happening:</b> Verify detectors are TRUE during green, <code>#MaxExt_*</code> &gt; 0, and <code>#ExtStep</code> &gt; 0.</li>
</ul>

<hr />

<h3>License & Use</h3>
<p>Educational/demo use. For real intersections, comply with local traffic signal standards and safety integrity requirements.</p>

</body>
</html>
