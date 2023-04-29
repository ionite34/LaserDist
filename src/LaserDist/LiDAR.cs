/*
 * Created in SharpDevelop.
 * User: Dunbaratu
 * Date: 7/3/2014
 * Time: 1:14 AM
 *
 * This file is part of LaserDist - a freely available module
 * for Kerbal Space Program.
 * Copyright (C) 2014 Steven Mading (aka user "Dunbaratu" on GitHub.)
 * author contact: madings@gmail.com
 *
 * This file, and the other files in this project, are distributed
 * under the terms of the GPL3 (Gnu Public License version 3).
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
using System;
using System.Collections.Generic;
using UnityEngine;

namespace LiDAR
{
    /// <summary>
    /// The class associated with the plugin to run the LiDAR
    /// part(s).
    /// </summary>
    public class LiDARModule : PartModule
    {

        private bool debugLineDraw = false;

        /// <summary>
        /// We want to do the work once per FixedUpdate(), but NOT during the
        /// midst of the Fixeduodate() since that means half the other parts in the game
        /// have moved their objects for the next tick and half haven't yet.  That
        /// causes raycasts to become unreliable and cofusing.
        /// So instead we just flag that a FixedUpdate() has occurred, and let the next Update()
        /// just after that FixedUpdate() do the work. (but if additional Update()s happen more often
        /// than FixedUpdates, the "extra" Update()s won't do the work.)
        /// </summary>
        private bool fixedUpdateHappened = false;

        /// <summary>
        ///   Laser's origin relative to the part's coord transform:
        /// </summary>
        static private Vector3d relLaserOrigin;
        /// <summary>
        ///   Laser's origin in Unity World coords:
        /// </summary>
        private Vector3d origin;
        /// <summary>
        ///   Laser's pointing unit vector in Unity World coords before x/y deflection was applied.
        /// </summary>
        private Vector3d rawPointing;
        /// <summary>
        ///   Laser's pointing unit vector in Unity World coords after x/y deflection has been applied.
        /// </summary>
        private Vector3d[] beamsPointing = new Vector3d[8];

        /// <summary>
        ///   Laser's origin in the map view's coords:
        /// </summary>
        private Vector3d mapOrigin;

        /// <summary>
        /// Remember the object that had been hit by bestUnityRayCastDist
        /// </summary>
        private RaycastHit bestLateUpdateHit = new RaycastHit();

        private bool isDrawing = false;
        private bool isOnMap = false;
        private bool isInEditor = false;
        private bool hasPower = false;
        private double prevTime = 0.0;
        private double deltaTime = 0.0;

        // These are settings that affect the color animation of the laser beam:

        private Color laserColor = new Color(1.0f, 0.0f, 0.0f);
        private float laserOpacityAverage = 0.45f;
        private float laserOpacityVariance = 0.20f;
        private float laserOpacityFadeMin = 0.1f; // min opacity when at max distance.
        private System.Random laserAnimationRandomizer = null;

        // This varies the "wowowow" laser thickness animation:
        private delegate float ThicknessTimeFunction(long millisec, int rand100);
        private ThicknessTimeFunction laserWidthTimeFunction = delegate (long ms, int rand100)
        {
            return 0.3f + 0.2f * (Mathf.Sin(ms / 200) + (rand100 / 100f) - 0.5f);
        };
        private System.Diagnostics.Stopwatch thicknessWatch;


        private GameObject[] lineObj = new GameObject[8];
        private LineRenderer[] line = new LineRenderer[8];
        private GameObject debuglineObj = null;
        private LineRenderer debugline = null;
        private Int32 mask;
        private Int32 laserFlightDrawLayer;
        private Int32 laserMapDrawLayer;
        private Int32 laserEditorDrawLayer;

        // Sensor origin (3), beam direction (3), distance to impact (1), number of beams (8) 
        private static int frameLength = 3 + (3 + 1) * 8;

        RaycastHit[] beams = new RaycastHit[8];

        /// Max number of values stored before cloud data is reset
        int MaxPoints = frameLength * 20;

        /// Tracks the number of ticks since last beam ray cast
        int CurrentTick = 0;

        /// <summary>
        ///  Number of game ticks to skip between beam ray casts
        /// </summary>
        [KSPField(isPersistant = true, guiName = "Tick Skip", guiActive = true, guiActiveEditor = true, guiUnits = "", guiFormat = "N2")]
        [UI_FloatRange(minValue = 0, maxValue = 40, stepIncrement = 1.0f)]
        public float TickSpacing = 10;

        /// <summary>PointCloud</summary>summary>
        public IList<double> cloudData = new List<double>();

        /// <summary>Distance the laser is checking to the first collision:</summary>
        [KSPField(isPersistant = true, guiName = "Max Sensor Range", guiActive = true, guiActiveEditor = true, guiUnits = "m")]
        public float MaxDistance = 10000f;

        /// <summary>Distance the laser is checking to the first collision:</summary>
        [KSPField(isPersistant = true, guiName = "Max Bend X", guiActive = true, guiActiveEditor = true, guiUnits = "deg")]
        public float MaxBendX = 0f;

        /// <summary>Distance the laser is checking to the first collision:</summary>
        [KSPField(isPersistant = true, guiName = "Max Bend Y", guiActive = true, guiActiveEditor = true, guiUnits = "deg")]
        public float MaxBendY = 0f;

        /// <summary>Flag controlling whether or not to see the laserbeam onscreen</summary>
        [KSPField(isPersistant = true, guiName = "Visible", guiActive = true, guiActiveEditor = true),
         UI_Toggle(disabledText = "no", enabledText = "yes")]
        public bool DrawLaser = false;

        /// <summary>Flag controlling whether or not the device is taking readings</summary>
        [KSPField(isPersistant = true, guiName = "Enabled", guiActive = true, guiActiveEditor = true),
         UI_Toggle(disabledText = "no", enabledText = "yes")]
        public bool Activated = false;

        /// <summary>Flag controlling whether or not the device is taking readings</summary>
        [KSPField(isPersistant = true, guiName = "Electricity", guiActive = true, guiActiveEditor = true),
         UI_Toggle(disabledText = "no", enabledText = "yes")]
        public bool Electric = false;

        /// <summary>electric usage per second that it's on:</summary>
        [KSPField(isPersistant = true, guiName = "Electricity Drain", guiActive = true, guiActiveEditor = true, guiUnits = "/sec", guiFormat = "N2")]
        public float ElectricPerSecond = 0.0f;

        /// <summary>How far to bend the laser beam relative to the part's "right" yaw. Negative values bend left.</summary>
        [KSPField(isPersistant = true, guiName = "Bend X", guiActive = false, guiActiveEditor = false, guiUnits = "deg", guiFormat = "N2")]
        [UI_FloatRange(minValue = 0, maxValue = 45, stepIncrement = 0.001f)]
        public float BendX = 0.0f;

        /// <summary>How far to bend the laser beam relative to the part's "up" pitch. Negative values bend down.</summary>
        [KSPField(isPersistant = true, guiName = "Bend Y", guiActive = false, guiActiveEditor = false, guiUnits = "deg", guiFormat = "N2")]
        [UI_FloatRange(minValue = -15, maxValue = 15, stepIncrement = 0.001f)]
        public float BendY = 0.0f;

        [KSPEvent(guiName = "Zero Bend", guiActive = false, guiActiveEditor = false)]
        public void ZeroBend()
        {
            BendX = 0f;
            BendY = 0f;
        }

        /// <summary>Flag controlling whether or not debug messages are enabled</summary>
        [KSPField(isPersistant = true, guiName = "Debug", guiActive = true, guiActiveEditor = true),
         UI_Toggle(disabledText = "no", enabledText = "yes")]
        public bool debugMsg = true;

        /// <summary>
        /// Configures context menu settings that vary depending on part.cfg settings per part,
        /// and therefore can't be configured in the C# attributes syntax (which is set in stone
        /// at compile time as static data that can't change per instance).
        /// </summary>
        private void SetGuiFieldsFromSettings()
        {
            // FIXME: The logic of the code below doesn't seem to be able to support having
            // multiple different instances of this PartModule that have different max and min
            // values for the float ranges.  It seems that whichever instance edited it's max/min
            // settings most recently, it ends up applying that to ALL the other ones too.
            // As far as I can tell it's acting like all the instances share the same UI_FloatRange
            // settings, like they're static for the class or something.
            // Strangely, this only seems to be a problem in the Flight view.  In the VAB/SPH,
            // I can actually get several different ranges on the rightclick menues.  But in the
            // Flight view, They're all the same, and it's always the settings for whichever of the
            // parts happens to have been loaded onto the vessel last.
            //
            // Because of this problem, for now I'm releasing the parts with all having the same
            // deflection range until I can understand this problem.

            BaseField field;

            DebugMsg("LiDAR is trying to config GUI panel fields from settings:");
            DebugMsg($"Part name = {part.name}, MaxBendX = {MaxBendX}, MaxBendY = {MaxBendY}");

            field = Fields["BendX"];
            ((UI_FloatRange)field.uiControlEditor).minValue = -MaxBendX;
            ((UI_FloatRange)field.uiControlEditor).maxValue = MaxBendX;
            if (MaxBendX == 0f)
            {
                field.guiActive = false;
                field.guiActiveEditor = false;
            }
            else
            {
                field.guiActive = true;
                field.guiActiveEditor = true;
            }

            field = Fields["BendY"];
            ((UI_FloatRange)field.uiControlEditor).minValue = -MaxBendY;
            ((UI_FloatRange)field.uiControlEditor).maxValue = MaxBendY;
            if (MaxBendY == 0f)
            {
                field.guiActive = false;
                field.guiActiveEditor = false;
            }
            else
            {
                field.guiActive = true;
                field.guiActiveEditor = true;
            }

            BaseEvent evt = Events["ZeroBend"];
            if (MaxBendX == 0f && MaxBendY == 0f)
            {
                evt.guiActive = false;
                evt.guiActiveEditor = false;
            }
            else
            {
                evt.guiActive = true;
                evt.guiActiveEditor = true;
            }
        }

        public override void OnStart(StartState state)
        {
            // Have to keep re-doing this from several hooks because
            // KSP keeps annoyingly forgetting my float range changes.
            SetGuiFieldsFromSettings();
        }

        /// <summary>
        /// Unity calls this hook during the activation of the partmodule on a part.
        /// </summary>
        /// <param name="state"></param>
        public override void OnAwake()
        {
            moduleName = "LiDARModule";
            relLaserOrigin = new Vector3d(0.0, 0.0, 0.0);

            SetGuiFieldsFromSettings();

            bool debugShowAllMaskNames = false; // turn on to print the following after a KSP update:
            if (debugShowAllMaskNames)
            {
                for (int i = 0; i < 32; i++)
                    System.Console.WriteLine("A layer called \"" + LayerMask.LayerToName(i) + "\" exists at bit position " + i);
            }
            // WARNING TO ANY FUTURE MAINTAINERS ABOUT THE FOLLOWING LAYERMASK SETTING:
            //
            // SQUAD does not put the layer mask values into any sort of an Enum I could find.
            // There isn't any guarantee that they'll keep the same names.  Therefore always
            // test this again after every KSP update to see if these values have
            // been changed or if more have been added.  LaserDist has been broken by
            // KSP updates in the past due to this being changed.  You can use the debug
            // printout in the lines above to see the new layer mask names after an update.
            // 
            // This is a bit-mask, but we don't have to do our own bit shifting to make it because
            // Unity provides the following string-name based way to build the mask.
            // The commented-out lines are present as a form of documentation.  It shows
            // what we're masking off - otherwise that would be unclear because those names
            // aren't mentioned elsewhere.
            mask = LayerMask.GetMask(
                "Default",    // layer number  0, which contains most physical objects that are not "scenery"
                              // "TransparentFX",    // layer number  1
                              // "Ignore Raycast",    // layer number  2
                              // "",    // layer number  3 (no name - don't know what it is)
                "Water",    // layer number  4
                            // "UI",    // layer number  5
                            // "",    // layer number  6 (no name - don't know what it is)
                            // "",    // layer number  7 (no name - don't know what it is)
                            // "PartsList_Icons",    // layer number  8
                            // "Atmosphere",    // layer number  9
                            // "Scaled Scenery",    // layer number  10 (this is the map view planets, I think)
                            // "UIDialog",    // layer number  11
                            // "UIVectors",    // layer number  12 (i.e. lines for orbits and comm connections maybe?)
                            // "UI_Mask",    // layer number  13
                            // "Screens",    // layer number  14
                "Local Scenery",    // layer number  15
                                    // "kerbals",    // layer number  16 (presumably the hovering faces in the UI, not the 3-D in-game kerbals)
                "EVA",    // layer number  17
                          // "SkySphere",    // layer number  18
                "PhysicalObjects",    // layer number  19 (don't know - maybe rocks?)
                                      // "Internal Space",    // layer number  20 (objects inside the cockpit in IVA view)
                                      // "Part Triggers",    // layer number  21 (don't know what this is)
                                      // "KerbalInstructors",    // layer number  22 (presumably the people's faces on screen?
                                      // "AeroFXIgnore",    // layer number  23 (well, it says "ignore" so I will)
                                      // "MapFX",    // layer number  24
                                      // "UIAdditional".    // layer number  25
                                      // "WheelCollidersIgnore",    // layer number  26
                "WheelColliders",    // layer number  27
                "TerrainColliders"    // layer number  28
                                      // "DragRender"    // layer number  29
                                      // "SurfaceFX"    // layer number  30
                                      // "Vectors"    // layer number  31 (UI overlay for things like lift and drag display, maybe?).
            );
            laserFlightDrawLayer = LayerMask.NameToLayer("TransparentFX");
            laserMapDrawLayer = LayerMask.NameToLayer("Scaled Scenery");
            laserEditorDrawLayer = LayerMask.NameToLayer("Default");
        }

        public override void OnActive()
        {
            GameEvents.onPartDestroyed.Add(OnLaserDestroy);
            GameEvents.onEditorShipModified.Add(OnLaserAttachDetach);
        }

        // Actions to control the active flag:
        // ----------------------------------------
        [KSPAction("toggle")]
        public void ActionToggle(KSPActionParam p)
        {
            Activated = !Activated;
        }

        // Actions to control the visibility flag:
        // ----------------------------------------
        [KSPAction("toggle visibility")]
        public void ActionToggleVisibility(KSPActionParam p)
        {
            DrawLaser = !DrawLaser;
        }

        public void ClearCloudData()
        {
            cloudData.Clear();
        }

        private void ChangeIsDrawing()
        {
            bool newVal = (hasPower && Activated && DrawLaser);
            if (newVal != isDrawing)
            {
                if (newVal)
                {
                    startDrawing();
                }
                else
                {
                    stopDrawing();
                }
            }

            isDrawing = newVal;
        }

        /// <summary>
        ///   Begin the Unity drawing of this laser,
        ///   making the unity objects for it.
        /// </summary>
        private void startDrawing()
        {
            for (int i = 0; i < beams.Length; ++i)
            {
                string name = "LiDAR beam" + i;
                lineObj[i] = new GameObject(name);
                isOnMap = MapView.MapIsEnabled;
                ChooseLayerBasedOnScene(i);

                Shader vecShader = Shader.Find("Particles/Alpha Blended"); // for when KSP version is < 1.8
                if (vecShader == null)
                    vecShader = Shader.Find("Legacy Shaders/Particles/Alpha Blended"); // for when KSP version is >= 1.8

                line[i] = lineObj[i].AddComponent<LineRenderer>();

                line[i].material = new Material(vecShader);
                Color c1 = laserColor;
                Color c2 = laserColor;
                line[i].startColor = c1;
                line[i].endColor = c2;
                line[i].enabled = true;

                laserAnimationRandomizer = new System.Random();
                bestLateUpdateHit.distance = -1f;

                if (thicknessWatch != null)
                    thicknessWatch.Stop();
                thicknessWatch = new System.Diagnostics.Stopwatch();
                thicknessWatch.Start();
            }
        }

        private void ChooseLayerBasedOnScene(int i)
        {
            isOnMap = MapView.MapIsEnabled;
            isInEditor = HighLogic.LoadedSceneIsEditor;
            Int32 newMask; // holding in a local var temporarily for debug-ability, because Unity overrides the value
                           // if it doesn't like it when you set LineObj.layer directly, making it hard to debug
                           // what's really going on becuase there's no variable value to look at which hasn't been altered.
            if (isInEditor)
            {
                newMask = laserEditorDrawLayer;
            }
            else if (isOnMap)
            {
                // Drawing the laser on the map was
                // only enabled for the purpose of debugging.
                // It might go away later:
                newMask = laserMapDrawLayer;
            }
            else
            {
                newMask = laserFlightDrawLayer;
            }
            lineObj[i].layer = newMask;
        }

        /// <summary>
        ///   Stop the Unity drawing of this laser:
        ///   destroying the unity objects for it.
        /// </summary>
        private void stopDrawing()
        {
            for (int i = 0; i < line.Length; ++i)
            {
                if (line[i] != null)
                {
                    line[i].enabled = false;
                    line[i] = null;
                }
                if (lineObj[i] != null)
                {
                    lineObj[i] = null;
                }
            }
        }

        /// <summary>
        /// Make sure to stop the beam picture as the part is blown up:
        /// </summary>
        public void OnLaserDestroy(Part p)
        {
            // To resolve github issue #5:
            // It turns out KSP will call this on ALL part destructions anywhere
            // in the game, not just when the part being destroyed is this one,
            // so don't do anything if it's the wrong part.
            //
            if (p != this.part) return;

            Activated = false;
            DrawLaser = false;
            ChangeIsDrawing();
        }

        public void OnDestroy()
        {
            OnLaserDestroy(this.part); // another way to catch it when a part is detached.
            GameEvents.onPartDestroyed.Remove(OnLaserDestroy);
            GameEvents.onEditorShipModified.Remove(OnLaserAttachDetach);
        }

        public void OnLaserAttachDetach(ShipConstruct sc)
        {
            // If this laser part isn't on the ship anymore, turn off the drawing.
            if (!sc.Parts.Contains(this.part))
                stopDrawing();
        }


        public void FixedUpdate()
        {
            fixedUpdateHappened = true;
        }

        /// <summary>
        /// Recalculate the pointing vector and origin point based on part current position and bending deflections.
        /// </summary>
        private void UpdatePointing(int i)
        {
            float ang = -BendX + i * (2 * BendX / (beams.Length - 1));
            if (MaxBendX > 0f || MaxBendY > 0f)
            {   // Doubles would be better than Floats here, but these come from user
                // interface rightclick menu fields that KSP demands be floats:
                Quaternion BendRotation =
                    Quaternion.AngleAxis(ang, this.part.transform.forward) *
                    Quaternion.AngleAxis(BendY, this.part.transform.right);
                beamsPointing[i] = BendRotation * rawPointing;
            }
            else
            {
                beamsPointing[i] = rawPointing;
            }
        }

        /// <summary>
        ///   Gets new distance reading if the device is on,
        ///   and handles the toggling of the display of the laser.
        /// </summary>
        public void Update()
        {
            if (!fixedUpdateHappened)
            {
                //DebugMsg("Update: a FixedUpdate hasn't happened yet, so skipping.");
                return;
            }
            //DebugMsg("Update: A new FixedUpdate happened, so doing the full work this time.");
            fixedUpdateHappened = false;

            double nowTime = Planetarium.GetUniversalTime();

            deltaTime = nowTime - prevTime;
            if (prevTime > 0) // Skips the power drain if it's the very first Update() after the scene load.
                drainPower();
            prevTime = nowTime;

            PhysicsRaycaster();
            ChangeIsDrawing();
            drawUpdate();

        }

        /// <summary>
        /// Use electriccharge, and check if power is out, and if so, disable:
        /// </summary>
        private void drainPower()
        {
            if (isInEditor)
            {
                hasPower = true;
            }
            else
            {
                if (Activated && Electric)
                {
                    double drainThisUpdate = (ElectricPerSecond * deltaTime);
                    double actuallyUsed = part.RequestResource("ElectricCharge", drainThisUpdate);
                    if (actuallyUsed < drainThisUpdate / 2.0)
                    {
                        hasPower = false;
                    }
                    else
                    {
                        hasPower = true;
                    }
                }
            }
        }

        /// <summary>
        /// Calculate rotation quaternion inverse for worldspace:
        /// </summary>

        private QuaternionD rotInv(CelestialBody body)
        {
            var up = body.bodyTransform.up;
            var forward = body.bodyTransform.forward;
            if (Math.Abs(Vector3d.Dot(up.normalized, forward.normalized)) > 0.1)
                throw new InvalidOperationException("Forward and up directions are not close to perpendicular, got " + up + " and " + forward);

            {
                forward.Normalize();
                up.Normalize(); // Additional normalization, avoids large tangent norm
                //var proj = forward * Vector3d.Dot(up, forward);
                var dot = Vector3d.Dot(up, forward);
                Vector3d proj = new Vector3d(dot * forward.x, dot * forward.y, dot * forward.z);
                up = up - proj;
                up.Normalize();
            }

            Vector3d right = Vector3d.Cross(up, forward);
            var w = Math.Sqrt(1.0d + right.x + up.y + forward.z) * 0.5d;
            var r = 0.25d / w;
            var x = (up.z - forward.y) * r;
            var y = (forward.x - right.z) * r;
            var z = (right.y - up.x) * r;

            return new QuaternionD(-x, -y, -z, w);
        }

        /// <summary>
        /// Perform Unity's Physics.RayCast() check:
        /// </summary>
        public void PhysicsRaycaster()
        {
            CurrentTick++;

            if (CurrentTick >= (int) TickSpacing)
                CurrentTick = 0;

            var partTransform = part.transform;
            origin = partTransform.TransformPoint(relLaserOrigin);
            rawPointing = partTransform.rotation * Vector3d.up;

            mapOrigin = ScaledSpace.LocalToScaledSpace(origin);

            // Get transform
            var currentBody = part.vessel.mainBody;
            var pos = currentBody.position;
            var rot = rotInv(currentBody);
            
            // Add sensor origin
            if (CurrentTick == 0 && cloudData.Count == 0)
            {
                var worldOrigin = rot * (origin - pos);
                cloudData.Add(worldOrigin.x);
                cloudData.Add(worldOrigin.y);
                cloudData.Add(worldOrigin.z);
                // DebugMsg(String.Format("Origin = {0} {1} {2}", origin.x, origin.y, origin.z));
                // DebugMsg(String.Format("MapOrigin = {0} {1} {2}", mapOrigin.x, mapOrigin.y, mapOrigin.z));
            }

            for (var i = 0; i < beams.Length; ++i)
            {
                UpdatePointing(i);

                var thisLateUpdateBestHit = new RaycastHit();

                if (hasPower && Activated && origin != null && beamsPointing[i] != null && CurrentTick == 0)
                {
                    var hits = new RaycastHit[] { };
                    Physics.RaycastNonAlloc(origin, beamsPointing[i], hits, MaxDistance, mask);

                    if (hits.Length > 0)
                    {
                        // Get the best existing hit on THIS lateUpdate:
                        thisLateUpdateBestHit.distance = Mathf.Infinity;
                        foreach (RaycastHit hit in hits)
                        {
                            if (hit.distance < thisLateUpdateBestHit.distance)
                                thisLateUpdateBestHit = hit;
                        }

                    }
                    else
                    {
                        //DebugMsg("  Raycast no hits.");
                        //if (updateForcedResultAge >= consecutiveForcedResultsAllowed)
                        {
                            thisLateUpdateBestHit = new RaycastHit(); // force it to count as a real miss.
                            thisLateUpdateBestHit.distance = -1f;
                        }
                    }

                    beams[i] = thisLateUpdateBestHit;

                    if (thisLateUpdateBestHit.distance > 0)
                    {
                        var worldPoint = rot * (beams[i].point - pos);
                        cloudData.Add(worldPoint.x);
                        cloudData.Add(worldPoint.y);
                        cloudData.Add(worldPoint.z);
                    }

                    // Add direction
                    //cloudData.Add(beamsPointing[i].x);
                    //cloudData.Add(beamsPointing[i].y);
                    //cloudData.Add(beamsPointing[i].z);

                    // Add distance
                    //cloudData.Add(thisLateUpdateBestHit.distance);
                }


                // If showing debug lines, this makes a purple line during LateUpdate
                // whenever the target changes to a new one:
                if (debugLineDraw)
                {
                    debuglineObj = new GameObject("LiDAR debug beam");
                    debuglineObj.layer = laserFlightDrawLayer;
                    debugline = debuglineObj.AddComponent<LineRenderer>();

                    debugline.material = new Material(Shader.Find("Particles/Additive"));
                    Color c1 = new Color(1.0f, 0.0f, 1.0f);
                    Color c2 = c1;
                    debugline.startColor = c1;
                    debugline.endColor = c2;
                    debugline.enabled = true;
                    debugline.startWidth = 0.01f;
                    debugline.endWidth = 0.01f;
                    debugline.SetPosition(0, origin);
                    debugline.SetPosition(1, origin + beamsPointing[i] * thisLateUpdateBestHit.distance);
                }
            }

            if (cloudData.Count > MaxPoints)
            {
                ClearCloudData();
                //DebugMsg("Point limit");
            }
        }

        /// <summary>
        ///   Draws the laser line to visually show the effect.
        ///   (Also useful for debugging).
        /// </summary>
        private void drawUpdate()
        {
            isOnMap = MapView.MapIsEnabled;
            isInEditor = HighLogic.LoadedSceneIsEditor;
            if (isDrawing)
            {
                Vector3d useOrigin = origin;
                if (isOnMap)
                {
                    useOrigin = mapOrigin;
                }

                float width = 0.02f;

                for (int i = 0; i < line.Length; ++i)
                {
                    line[i].positionCount = 2;
                    line[i].SetPosition(0, useOrigin);
                    line[i].SetPosition(1, useOrigin + beamsPointing[i] * ((beams[i].distance > 0) ? beams[i].distance : MaxDistance));

                    // Make an animation effect where the laser's opacity varies on a sine-wave-over-time pattern:
                    Color c1 = laserColor;
                    Color c2 = laserColor;
                    c1.a = laserOpacityAverage + laserOpacityVariance * (laserAnimationRandomizer.Next(0, 100) / 100f);
                    c2.a = laserOpacityFadeMin;
                    line[i].startColor = c1;
                    line[i].endColor = c2;
                    float tempWidth = width * laserWidthTimeFunction(thicknessWatch.ElapsedMilliseconds, laserAnimationRandomizer.Next(0, 100));
                    line[i].startWidth = tempWidth;
                    line[i].endWidth = tempWidth;
                }
            }
        }

        private void DebugMsg(string message)
        {
            if (debugMsg)
                Debug.Log(message);
        }
    }
}
