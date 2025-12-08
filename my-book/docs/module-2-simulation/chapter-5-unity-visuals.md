---
title: "Unity for realistic visuals"
description: "Using Unity for high-fidelity visual simulation of humanoid robots"
learning_objectives:
  - "Set up Unity for humanoid robot simulation"
  - "Import and configure humanoid models in Unity"
  - "Implement realistic rendering and lighting"
  - "Integrate Unity with ROS2 for robot simulation"
---

# Unity for realistic visuals

## Learning Objectives

By the end of this chapter, you will be able to:
- Set up Unity for humanoid robot simulation
- Import and configure humanoid models in Unity
- Implement realistic rendering and lighting
- Integrate Unity with ROS2 for robot simulation

## Introduction

Unity is a powerful 3D development platform that excels at creating photorealistic visual simulations. For humanoid robotics, Unity provides advanced rendering capabilities, realistic physics, and sophisticated animation systems that can enhance the visual quality of robot simulation beyond what traditional robotics simulators offer. This chapter will guide you through setting up Unity for humanoid robot simulation and integrating it with ROS2.

## Setting up Unity for Robotics

### Installing Unity Hub and Unity Editor

1. **Download Unity Hub** from the Unity website
2. **Install Unity Editor** with the following modules:
   - Android Build Support (for mobile deployment)
   - Linux Build Support (for Linux deployment)
   - Universal Windows Platform Build Support
   - Visual Studio support for Unity

3. **Create a new 3D project** for your humanoid robot simulation

### Installing Unity Robotics Packages

Unity provides specialized packages for robotics simulation:

1. **Unity Robotics Hub**: A package manager for robotics tools
2. **Unity Robotics Simulation (URS)**: High-fidelity physics simulation
3. **ROS-TCP-Connector**: For ROS2 communication
4. **Unity Perception**: For synthetic data generation

```bash
# In Unity Package Manager, install:
# - ROS TCP Connector
# - Unity Perception
# - Unity Robotics Simulation (if available)
```

### Unity ROS-TCP-Connector Setup

The ROS-TCP-Connector enables communication between Unity and ROS2:

```csharp
// Create a new C# script: RosConnector.cs
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;

public class RosConnector : MonoBehaviour
{
    ROSConnection ros;
    string rosIPAddress = "127.0.0.1";
    int rosPort = 10000;

    // Start is called before the first frame update
    void Start()
    {
        // Get the ROS connection static instance
        ros = ROSConnection.instance;
        ros.RegisterPublisher<UInt8MultiArrayMsg>("/unity_robot_joint_commands");

        // Connect to ROS on the specified IP and port
        ros.Initialize(rosIPAddress, rosPort);
    }

    void Update()
    {
        // Send data to ROS
        SendJointData();
    }

    void SendJointData()
    {
        // Example: Send joint positions to ROS
        var jointData = new UInt8MultiArrayMsg();
        // Populate with actual joint data
        ros.Send("/unity_robot_joint_states", jointData);
    }
}
```

## Creating Humanoid Robot Models in Unity

### Importing Robot Models

Unity supports various 3D model formats (FBX, OBJ, DAE, etc.):

```csharp
// RobotController.cs - Basic robot controller script
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RobotController : MonoBehaviour
{
    [Header("Joint Configuration")]
    public List<Transform> jointTransforms = new List<Transform>();
    public List<ArticulationBody> jointArticulationBodies = new List<ArticulationBody>();

    [Header("Robot Parameters")]
    public float jointSpeed = 1.0f;
    public float maxTorque = 100.0f;

    // Start is called before the first frame update
    void Start()
    {
        SetupRobotJoints();
    }

    void SetupRobotJoints()
    {
        // Find all joint articulation bodies in the robot
        ArticulationBody[] bodies = GetComponentsInChildren<ArticulationBody>();
        jointArticulationBodies.AddRange(bodies);

        // Configure each joint
        foreach (ArticulationBody body in jointArticulationBodies)
        {
            ConfigureJoint(body);
        }
    }

    void ConfigureJoint(ArticulationBody joint)
    {
        // Set joint drive parameters
        ArticulationDrive drive = joint.xDrive;
        drive.forceLimit = maxTorque;
        drive.damping = 10.0f;
        drive.stiffness = 100.0f;
        joint.xDrive = drive;

        // Apply to all drives (x, y, z)
        joint.yDrive = drive;
        joint.zDrive = drive;
    }

    public void SetJointPositions(List<float> positions)
    {
        if (positions.Count != jointArticulationBodies.Count)
        {
            Debug.LogError("Joint position count mismatch!");
            return;
        }

        for (int i = 0; i < jointArticulationBodies.Count; i++)
        {
            ArticulationBody joint = jointArticulationBodies[i];

            // Set target position for the joint
            ArticulationDrive drive = joint.xDrive;
            drive.target = positions[i];
            joint.xDrive = drive;
        }
    }
}
```

### Creating a Humanoid Skeleton

For realistic humanoid animation, create a proper skeleton:

```csharp
// HumanoidSkeleton.cs
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[System.Serializable]
public class HumanoidJoint
{
    public string name;
    public Transform transform;
    public ArticulationBody articulationBody;
    public float minAngle;
    public float maxAngle;
    public float currentAngle;
}

public class HumanoidSkeleton : MonoBehaviour
{
    [Header("Skeleton Configuration")]
    public HumanoidJoint head;
    public HumanoidJoint neck;

    public List<HumanoidJoint> leftArm = new List<HumanoidJoint>();
    public List<HumanoidJoint> rightArm = new List<HumanoidJoint>();
    public List<HumanoidJoint> leftLeg = new List<HumanoidJoint>();
    public List<HumanoidJoint> rightLeg = new List<HumanoidJoint>();

    [Header("IK Configuration")]
    public bool useInverseKinematics = true;
    public Transform leftHandTarget;
    public Transform rightHandTarget;
    public Transform leftFootTarget;
    public Transform rightFootTarget;

    void Start()
    {
        InitializeSkeleton();
    }

    void InitializeSkeleton()
    {
        // Initialize all joints
        InitializeJoint(ref head, "head_joint");
        InitializeJoint(ref neck, "neck_joint");

        // Initialize limb joints
        InitializeLimbJoints("left_arm", ref leftArm);
        InitializeLimbJoints("right_arm", ref rightArm);
        InitializeLimbJoints("left_leg", ref leftLeg);
        InitializeLimbJoints("right_leg", ref rightLeg);
    }

    void InitializeJoint(ref HumanoidJoint joint, string jointName)
    {
        Transform jointTransform = transform.Find(jointName);
        if (jointTransform != null)
        {
            joint = new HumanoidJoint
            {
                name = jointName,
                transform = jointTransform,
                articulationBody = jointTransform.GetComponent<ArticulationBody>()
            };
        }
    }

    void InitializeLimbJoints(string limbPrefix, ref List<HumanoidJoint> limbJoints)
    {
        // Find all joints with the limb prefix
        Transform limbRoot = transform.Find(limbPrefix);
        if (limbRoot != null)
        {
            foreach (Transform child in limbRoot)
            {
                HumanoidJoint joint = new HumanoidJoint
                {
                    name = child.name,
                    transform = child,
                    articulationBody = child.GetComponent<ArticulationBody>()
                };
                limbJoints.Add(joint);
            }
        }
    }

    public void SetJointAngles(Dictionary<string, float> jointAngles)
    {
        SetJointAngle(head, jointAngles);
        SetJointAngle(neck, jointAngles);

        SetLimbJointAngles(leftArm, jointAngles);
        SetLimbJointAngles(rightArm, jointAngles);
        SetLimbJointAngles(leftLeg, jointAngles);
        SetLimbJointAngles(rightLeg, jointAngles);
    }

    void SetJointAngle(HumanoidJoint joint, Dictionary<string, float> angles)
    {
        if (joint != null && angles.ContainsKey(joint.name))
        {
            float targetAngle = Mathf.Clamp(angles[joint.name], joint.minAngle, joint.maxAngle);
            joint.currentAngle = targetAngle;

            if (joint.articulationBody != null)
            {
                ArticulationDrive drive = joint.articulationBody.xDrive;
                drive.target = targetAngle;
                joint.articulationBody.xDrive = drive;
            }
        }
    }

    void SetLimbJointAngles(List<HumanoidJoint> limb, Dictionary<string, float> angles)
    {
        foreach (HumanoidJoint joint in limb)
        {
            SetJointAngle(joint, angles);
        }
    }
}
```

## Realistic Rendering and Lighting

### Setting up High-Quality Rendering

Unity's Built-in Render Pipeline or Universal Render Pipeline (URP) can be configured for realistic visuals:

```csharp
// RobotRenderer.cs - Advanced rendering configuration
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;

public class RobotRenderer : MonoBehaviour
{
    [Header("Material Configuration")]
    public Material robotBodyMaterial;
    public Material jointMaterial;
    public Material sensorMaterial;

    [Header("Lighting Settings")]
    public Light mainLight;
    public float ambientIntensity = 0.2f;
    public Color ambientColor = Color.gray;

    [Header("Reflection Settings")]
    public ReflectionProbe robotReflectionProbe;
    public float reflectionIntensity = 1.0f;

    [Header("Post-Processing")]
    public bool enablePostProcessing = true;
    public float bloomIntensity = 0.5f;
    public float chromaticAberration = 0.1f;

    void Start()
    {
        ConfigureRendering();
        SetupLighting();
    }

    void ConfigureRendering()
    {
        // Apply materials to robot parts
        ApplyMaterials();

        // Configure reflection probes for realistic reflections
        ConfigureReflectionProbes();

        // Set up shadows and lighting
        ConfigureShadows();
    }

    void ApplyMaterials()
    {
        Renderer[] renderers = GetComponentsInChildren<Renderer>();

        foreach (Renderer renderer in renderers)
        {
            if (renderer.name.Contains("joint"))
            {
                renderer.material = jointMaterial;
            }
            else if (renderer.name.Contains("sensor"))
            {
                renderer.material = sensorMaterial;
            }
            else
            {
                renderer.material = robotBodyMaterial;
            }
        }
    }

    void ConfigureReflectionProbes()
    {
        if (robotReflectionProbe != null)
        {
            robotReflectionProbe.intensity = reflectionIntensity;
            robotReflectionProbe.mode = ReflectionProbeMode.Realtime;
        }
    }

    void ConfigureShadows()
    {
        // Configure shadow settings for realistic lighting
        QualitySettings.shadowDistance = 50f;
        QualitySettings.shadowResolution = ShadowResolution.High;
        QualitySettings.shadowProjection = ShadowProjection.StableFit;
    }

    void SetupLighting()
    {
        // Configure main directional light
        if (mainLight != null)
        {
            mainLight.shadows = LightShadows.Soft;
            mainLight.shadowStrength = 0.8f;
            mainLight.shadowBias = 0.2f;
            mainLight.shadowNormalBias = 0.5f;
        }

        // Set ambient lighting
        RenderSettings.ambientIntensity = ambientIntensity;
        RenderSettings.ambientLight = ambientColor;
        RenderSettings.ambientMode = UnityEngine.Rendering.AmbientMode.Trilight;
    }
}
```

### Creating Realistic Materials

Create materials that simulate real robot components:

```csharp
// MaterialManager.cs - Dynamic material management
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MaterialManager : MonoBehaviour
{
    [Header("Material Presets")]
    public Material aluminumMaterial;
    public Material steelMaterial;
    public Material rubberMaterial;
    public Material ledMaterial;

    [Header("Dynamic Properties")]
    public float surfaceRoughness = 0.3f;
    public float metallicValue = 0.8f;
    public Color baseColor = Color.gray;

    void Start()
    {
        CreateDynamicMaterials();
    }

    void CreateDynamicMaterials()
    {
        // Create aluminum material
        if (aluminumMaterial == null)
        {
            aluminumMaterial = new Material(Shader.Find("Standard"));
            aluminumMaterial.name = "Aluminum";
            aluminumMaterial.SetColor("_Color", new Color(0.8f, 0.8f, 0.85f));
            aluminumMaterial.SetFloat("_Metallic", 0.9f);
            aluminumMaterial.SetFloat("_Smoothness", 0.7f);
        }

        // Create steel material
        if (steelMaterial == null)
        {
            steelMaterial = new Material(Shader.Find("Standard"));
            steelMaterial.name = "Steel";
            steelMaterial.SetColor("_Color", new Color(0.6f, 0.6f, 0.7f));
            steelMaterial.SetFloat("_Metallic", 0.7f);
            steelMaterial.SetFloat("_Smoothness", 0.5f);
        }

        // Create rubber material for feet
        if (rubberMaterial == null)
        {
            rubberMaterial = new Material(Shader.Find("Standard"));
            rubberMaterial.name = "Rubber";
            rubberMaterial.SetColor("_Color", Color.black);
            rubberMaterial.SetFloat("_Metallic", 0.0f);
            rubberMaterial.SetFloat("_Smoothness", 0.1f);
        }

        // Create LED material for sensors
        if (ledMaterial == null)
        {
            ledMaterial = new Material(Shader.Find("Standard"));
            ledMaterial.name = "LED";
            ledMaterial.SetColor("_Color", Color.red);
            ledMaterial.SetFloat("_Metallic", 0.2f);
            ledMaterial.SetFloat("_Smoothness", 0.9f);
            ledMaterial.EnableKeyword("_EMISSION");
            ledMaterial.SetColor("_EmissionColor", Color.red * 0.5f);
        }
    }

    public Material CreateCustomMaterial(Color color, float metallic, float smoothness)
    {
        Material customMaterial = new Material(Shader.Find("Standard"));
        customMaterial.SetColor("_Color", color);
        customMaterial.SetFloat("_Metallic", metallic);
        customMaterial.SetFloat("_Smoothness", smoothness);

        return customMaterial;
    }

    public void ApplyMaterialToPart(string partName, Material material)
    {
        Transform part = transform.Find(partName);
        if (part != null)
        {
            Renderer renderer = part.GetComponent<Renderer>();
            if (renderer != null)
            {
                renderer.material = material;
            }
        }
    }
}
```

## Environment and Scene Setup

### Creating Realistic Environments

```csharp
// EnvironmentManager.cs - Scene environment setup
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class EnvironmentManager : MonoBehaviour
{
    [Header("Environment Configuration")]
    public GameObject[] environmentPrefabs;
    public Material[] groundMaterials;
    public Light[] sceneLights;

    [Header("Weather Effects")]
    public bool enableFog = false;
    public Color fogColor = Color.gray;
    public float fogDensity = 0.01f;

    [Header("Sky Configuration")]
    public Material skyboxMaterial;
    public Gradient skyGradient;

    void Start()
    {
        SetupEnvironment();
        ConfigureAtmosphere();
        SetupCameras();
    }

    void SetupEnvironment()
    {
        // Set up ground/floor
        SetupGround();

        // Add environmental objects
        SpawnEnvironmentObjects();

        // Configure scene lighting
        SetupSceneLighting();
    }

    void SetupGround()
    {
        // Create or configure ground plane
        GameObject ground = GameObject.Find("Ground") ?? new GameObject("Ground");
        ground.transform.position = Vector3.zero;
        ground.transform.rotation = Quaternion.identity;

        // Add mesh renderer and collider
        if (!ground.GetComponent<MeshFilter>())
        {
            ground.AddComponent<MeshFilter>();
            ground.GetComponent<MeshFilter>().mesh = CreateGroundPlane();
        }

        if (!ground.GetComponent<MeshRenderer>())
        {
            ground.AddComponent<MeshRenderer>();
        }

        if (!ground.GetComponent<BoxCollider>())
        {
            ground.AddComponent<BoxCollider>();
        }

        // Apply ground material
        if (groundMaterials.Length > 0)
        {
            ground.GetComponent<MeshRenderer>().material = groundMaterials[0];
        }
    }

    Mesh CreateGroundPlane()
    {
        // Create a simple plane mesh
        Mesh mesh = new Mesh();

        Vector3[] vertices = new Vector3[4]
        {
            new Vector3(-50f, 0, -50f),
            new Vector3(50f, 0, -50f),
            new Vector3(-50f, 0, 50f),
            new Vector3(50f, 0, 50f)
        };

        int[] triangles = new int[6]
        {
            0, 1, 2,
            1, 3, 2
        };

        mesh.vertices = vertices;
        mesh.triangles = triangles;
        mesh.RecalculateNormals();

        return mesh;
    }

    void SpawnEnvironmentObjects()
    {
        // Randomly place environment objects
        foreach (GameObject prefab in environmentPrefabs)
        {
            for (int i = 0; i < 5; i++) // Spawn 5 of each type
            {
                Vector3 position = new Vector3(
                    Random.Range(-40f, 40f),
                    0,
                    Random.Range(-40f, 40f)
                );

                GameObject obj = Instantiate(prefab, position, Quaternion.identity);
                obj.transform.SetParent(transform);
            }
        }
    }

    void SetupSceneLighting()
    {
        // Configure scene lights
        foreach (Light light in sceneLights)
        {
            if (light.type == LightType.Directional)
            {
                // Set up main directional light (sun)
                light.shadows = LightShadows.Soft;
                light.shadowStrength = 0.8f;
                light.intensity = 1.0f;
            }
        }
    }

    void ConfigureAtmosphere()
    {
        // Set up fog
        RenderSettings.fog = enableFog;
        RenderSettings.fogColor = fogColor;
        RenderSettings.fogDensity = fogDensity;
        RenderSettings.fogMode = FogMode.ExponentialSquared;

        // Set up skybox
        if (skyboxMaterial != null)
        {
            RenderSettings.skybox = skyboxMaterial;
        }
    }

    void SetupCameras()
    {
        // Configure main camera
        Camera mainCam = Camera.main;
        if (mainCam != null)
        {
            mainCam.fieldOfView = 60f;
            mainCam.nearClipPlane = 0.1f;
            mainCam.farClipPlane = 1000f;
        }
    }
}
```

## Integration with ROS2

### ROS2 Communication Setup

```csharp
// Ros2Integrator.cs - ROS2 integration for Unity
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;

public class Ros2Integrator : MonoBehaviour
{
    [Header("ROS2 Configuration")]
    public string rosIPAddress = "127.0.0.1";
    public int rosPort = 10000;

    [Header("Topic Configuration")]
    public string jointStateTopic = "/unity_robot/joint_states";
    public string imageTopic = "/unity_robot/rgb/image_raw";
    public string cameraInfoTopic = "/unity_robot/rgb/camera_info";
    public string imuTopic = "/unity_robot/imu/data";

    private ROSConnection ros;
    private RobotController robotController;
    private Camera robotCamera;

    void Start()
    {
        // Initialize ROS connection
        ros = ROSConnection.instance;
        ros.Initialize(rosIPAddress, rosPort);

        // Get references to other components
        robotController = GetComponent<RobotController>();
        robotCamera = GetComponentInChildren<Camera>();

        // Register publishers
        ros.RegisterPublisher<JointStateMsg>(jointStateTopic);
        ros.RegisterPublisher<ImageMsg>(imageTopic);
        ros.RegisterPublisher<CameraInfoMsg>(cameraInfoTopic);
        ros.RegisterPublisher<ImuMsg>(imuTopic);

        // Start coroutines for data publishing
        StartCoroutine(PublishJointStates());
        StartCoroutine(PublishCameraData());
        StartCoroutine(PublishImuData());
    }

    IEnumerator PublishJointStates()
    {
        while (true)
        {
            // Create joint state message
            var jointState = new JointStateMsg();
            jointState.header = new HeaderMsg();
            jointState.header.stamp = new TimeMsg(System.DateTime.UtcNow);
            jointState.header.frame_id = "unity_robot";

            // Get joint names and positions from the robot controller
            jointState.name = GetJointNames();
            jointState.position = GetJointPositions();
            jointState.velocity = GetJointVelocities();
            jointState.effort = GetJointEfforts();

            // Publish the message
            ros.Publish(jointStateTopic, jointState);

            yield return new WaitForSeconds(0.02f); // 50Hz
        }
    }

    IEnumerator PublishCameraData()
    {
        while (true)
        {
            if (robotCamera != null)
            {
                // Capture image from camera
                Texture2D imageTexture = CaptureCameraImage(robotCamera);

                // Create image message
                var imageMsg = new ImageMsg();
                imageMsg.header = new HeaderMsg();
                imageMsg.header.stamp = new TimeMsg(System.DateTime.UtcNow);
                imageMsg.header.frame_id = "rgb_camera_link";

                imageMsg.height = (uint)imageTexture.height;
                imageMsg.width = (uint)imageTexture.width;
                imageMsg.encoding = "rgb8";
                imageMsg.is_bigendian = 0;
                imageMsg.step = (uint)(imageTexture.width * 3); // 3 bytes per pixel for RGB

                // Convert texture to byte array
                byte[] imageData = imageTexture.GetRawTextureData<byte>();
                imageMsg.data = imageData;

                // Publish image
                ros.Publish(imageTopic, imageMsg);

                // Also publish camera info
                PublishCameraInfo();
            }

            yield return new WaitForSeconds(0.033f); // ~30fps
        }
    }

    IEnumerator PublishImuData()
    {
        while (true)
        {
            // Create IMU message
            var imuMsg = new ImuMsg();
            imuMsg.header = new HeaderMsg();
            imuMsg.header.stamp = new TimeMsg(System.DateTime.UtcNow);
            imuMsg.header.frame_id = "imu_link";

            // Set orientation (from robot's rotation)
            imuMsg.orientation = ConvertQuaternion(transform.rotation);

            // Set angular velocity (simplified)
            imuMsg.angular_velocity = new Vector3Msg(0, 0, 0); // TODO: Calculate from joint velocities

            // Set linear acceleration (including gravity)
            imuMsg.linear_acceleration = new Vector3Msg(0, -9.81f, 0); // Gravity

            // Publish IMU data
            ros.Publish(imuTopic, imuMsg);

            yield return new WaitForSeconds(0.01f); // 100Hz
        }
    }

    List<string> GetJointNames()
    {
        // Return list of joint names
        // This should match your URDF joint names
        return new List<string>
        {
            "left_hip_joint", "left_knee_joint", "left_ankle_joint",
            "right_hip_joint", "right_knee_joint", "right_ankle_joint",
            "left_shoulder_joint", "left_elbow_joint", "left_wrist_joint",
            "right_shoulder_joint", "right_elbow_joint", "right_wrist_joint"
        };
    }

    float[] GetJointPositions()
    {
        // Get current joint positions from the robot controller
        // This is a simplified example - in practice, you'd get actual joint values
        float[] positions = new float[12];
        for (int i = 0; i < positions.Length; i++)
        {
            positions[i] = 0.0f; // Placeholder - get actual values from joints
        }
        return positions;
    }

    float[] GetJointVelocities()
    {
        // Get joint velocities
        float[] velocities = new float[12];
        for (int i = 0; i < velocities.Length; i++)
        {
            velocities[i] = 0.0f; // Placeholder
        }
        return velocities;
    }

    float[] GetJointEfforts()
    {
        // Get joint efforts/torques
        float[] efforts = new float[12];
        for (int i = 0; i < efforts.Length; i++)
        {
            efforts[i] = 0.0f; // Placeholder
        }
        return efforts;
    }

    Texture2D CaptureCameraImage(Camera cam)
    {
        // Capture image from camera
        RenderTexture currentRT = RenderTexture.active;
        RenderTexture.active = cam.targetTexture;

        cam.Render();

        Texture2D imageTexture = new Texture2D(cam.targetTexture.width, cam.targetTexture.height);
        imageTexture.ReadPixels(new Rect(0, 0, cam.targetTexture.width, cam.targetTexture.height), 0, 0);
        imageTexture.Apply();

        RenderTexture.active = currentRT;
        return imageTexture;
    }

    void PublishCameraInfo()
    {
        // Create and publish camera info message
        var cameraInfo = new CameraInfoMsg();
        cameraInfo.header = new HeaderMsg();
        cameraInfo.header.stamp = new TimeMsg(System.DateTime.UtcNow);
        cameraInfo.header.frame_id = "rgb_camera_link";

        cameraInfo.height = 480;
        cameraInfo.width = 640;
        cameraInfo.distortion_model = "plumb_bob";

        // Camera matrix (intrinsic parameters)
        cameraInfo.K = new double[] { 554.256, 0, 320.5, 0, 554.256, 240.5, 0, 0, 1 };

        // Rectification matrix
        cameraInfo.R = new double[] { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

        // Projection matrix
        cameraInfo.P = new double[] { 554.256, 0, 320.5, 0, 0, 554.256, 240.5, 0, 0, 0, 1, 0 };

        ros.Publish(cameraInfoTopic, cameraInfo);
    }

    GeometryMsgsQuaternion ConvertQuaternion(Quaternion q)
    {
        return new GeometryMsgsQuaternion(q.x, q.y, q.z, q.w);
    }
}
```

## Unity Perception Integration

Unity Perception can generate synthetic training data:

```csharp
// PerceptionManager.cs - Unity Perception integration
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Perception.GroundTruth;
using Unity.Perception.Randomization.Samplers;

public class PerceptionManager : MonoBehaviour
{
    [Header("Perception Configuration")]
    public bool enableGroundTruth = true;
    public float captureInterval = 0.5f;
    public int maxTrainingSamples = 10000;

    [Header("Sensor Configuration")]
    public Camera segmentationCamera;
    public GameObject[] objectsToTrack;

    [Header("Randomization")]
    public bool enableRandomization = true;
    public float lightingVariation = 0.5f;
    public float textureVariation = 0.3f;

    private int sampleCount = 0;
    private float lastCaptureTime = 0f;

    void Start()
    {
        if (enableGroundTruth)
        {
            SetupGroundTruth();
        }

        if (enableRandomization)
        {
            SetupRandomization();
        }
    }

    void SetupGroundTruth()
    {
        // Add ground truth components to tracked objects
        foreach (GameObject obj in objectsToTrack)
        {
            if (obj.GetComponent<GroundTruthSingleton>() == null)
            {
                obj.AddComponent<GroundTruthSingleton>();
            }

            // Add segmentation labels
            if (obj.GetComponent<Labeling>() == null)
            {
                var labeling = obj.AddComponent<Labeling>();
                labeling.label = obj.name; // Use object name as label
            }
        }

        // Configure the segmentation camera
        if (segmentationCamera != null)
        {
            var segmentationSensor = segmentationCamera.gameObject.AddComponent<SegmentationSensor>();
            segmentationSensor.captureEveryNthFrame = 10; // Every 10 frames
        }
    }

    void SetupRandomization()
    {
        // Randomize lighting conditions
        StartCoroutine(RandomizeEnvironment());
    }

    IEnumerator RandomizeEnvironment()
    {
        while (true)
        {
            if (enableRandomization)
            {
                RandomizeLighting();
                RandomizeTextures();
                RandomizeObjectPositions();
            }

            yield return new WaitForSeconds(captureInterval);
        }
    }

    void RandomizeLighting()
    {
        // Randomize main light properties
        Light[] lights = FindObjectsOfType<Light>();
        foreach (Light light in lights)
        {
            if (light.type == LightType.Directional)
            {
                // Randomize light direction slightly
                Vector3 randomDirection = Random.insideUnitSphere * lightingVariation;
                light.transform.rotation = Quaternion.Euler(
                    light.transform.rotation.eulerAngles + new Vector3(
                        randomDirection.x * 10,
                        randomDirection.y * 10,
                        randomDirection.z * 10
                    )
                );

                // Randomize light intensity
                light.intensity = 1.0f + Random.Range(-lightingVariation, lightingVariation);
            }
        }
    }

    void RandomizeTextures()
    {
        // Randomize materials on objects
        Renderer[] renderers = FindObjectsOfType<Renderer>();
        foreach (Renderer renderer in renderers)
        {
            if (Random.value < textureVariation)
            {
                // Apply random color variation
                Color randomColor = renderer.material.color + new Color(
                    Random.Range(-0.1f, 0.1f),
                    Random.Range(-0.1f, 0.1f),
                    Random.Range(-0.1f, 0.1f)
                );
                renderer.material.color = randomColor;
            }
        }
    }

    void RandomizeObjectPositions()
    {
        // Randomly move objects slightly
        foreach (GameObject obj in objectsToTrack)
        {
            if (obj.CompareTag("Randomizable"))
            {
                Vector3 randomOffset = new Vector3(
                    Random.Range(-0.1f, 0.1f),
                    Random.Range(-0.1f, 0.1f),
                    Random.Range(-0.1f, 0.1f)
                );
                obj.transform.position += randomOffset;
            }
        }
    }

    void Update()
    {
        // Capture data at specified intervals
        if (enableGroundTruth && Time.time - lastCaptureTime >= captureInterval)
        {
            CaptureGroundTruthData();
            lastCaptureTime = Time.time;
            sampleCount++;

            if (sampleCount >= maxTrainingSamples)
            {
                Debug.Log("Reached maximum training samples. Stopping data capture.");
                enabled = false;
            }
        }
    }

    void CaptureGroundTruthData()
    {
        // This would trigger data capture in Unity Perception
        // The actual implementation depends on Unity Perception package
        Debug.Log($"Captured ground truth data sample #{sampleCount}");
    }
}
```

## Practical Exercise: Unity Humanoid Robot Simulation

Create a complete Unity scene with a humanoid robot:

1. **Create a new Unity 3D project** called "HumanoidRobotSimulation"

2. **Import the Unity Robotics packages**:
   - ROS TCP Connector
   - Unity Perception (if needed)

3. **Create the robot hierarchy**:
   ```
   HumanoidRobot
   ├── Torso
   │   ├── Head
   │   ├── LeftArm
   │   │   ├── Shoulder
   │   │   ├── Elbow
   │   │   └── Wrist
   │   ├── RightArm
   │   │   ├── Shoulder
   │   │   ├── Elbow
   │   │   └── Wrist
   │   ├── LeftLeg
   │   │   ├── Hip
   │   │   ├── Knee
   │   │   └── Ankle
   │   └── RightLeg
   │       ├── Hip
   │       ├── Knee
   │       └── Ankle
   ```

4. **Add components to the main robot object**:
   - RobotController
   - Ros2Integrator
   - RobotRenderer
   - MaterialManager

5. **Set up the scene**:
   - Add lighting
   - Add environment
   - Configure camera

6. **Test the simulation**:
   ```bash
   # Start ROS2 bridge
   ros2 run rosbridge_server rosbridge_websocket

   # Or use the TCP connector directly
   # Run Unity scene and check if data is being published
   ros2 topic list
   ros2 topic echo /unity_robot/joint_states
   ```

## Performance Optimization

### Unity Performance Settings for Robotics

```csharp
// PerformanceOptimizer.cs - Optimize Unity for robotics simulation
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PerformanceOptimizer : MonoBehaviour
{
    [Header("Performance Settings")]
    public bool enableLOD = true;
    public int targetFrameRate = 60;
    public bool enableOcclusionCulling = true;
    public bool enableDynamicBatching = true;
    public bool enableStaticBatching = true;

    [Header("Quality Settings")]
    public int shadowResolution = 1; // 0=Low, 1=Medium, 2=High, 3=Very High
    public int textureQuality = 1;   // 0=Low, 1=Medium, 2=High

    void Start()
    {
        ConfigurePerformanceSettings();
        OptimizeRendering();
    }

    void ConfigurePerformanceSettings()
    {
        // Set target frame rate
        Application.targetFrameRate = targetFrameRate;

        // Configure quality settings
        QualitySettings.shadowResolution = (ShadowResolution)shadowResolution;
        QualitySettings.masterTextureLimit = textureQuality;

        // Enable/disable batching
        StaticBatchingUtility.SetOptimized(true);
    }

    void OptimizeRendering()
    {
        // Configure occlusion culling if enabled
        if (enableOcclusionCulling)
        {
            // This needs to be set up in the scene view in Unity Editor
            Camera.main.layerCullDistances = new float[32]; // Configure per layer
        }

        // Use Level of Detail (LOD) for complex models
        if (enableLOD)
        {
            SetupLODGroups();
        }
    }

    void SetupLODGroups()
    {
        // Add LOD groups to complex robot parts
        LODGroup[] lodGroups = GetComponentsInChildren<LODGroup>();
        foreach (LODGroup lodGroup in lodGroups)
        {
            // Configure LOD levels
            LOD[] lods = new LOD[3];
            // Level 0: High detail (100%)
            lods[0] = new LOD(1.0f, lodGroup.GetLODs()[0].renderers);
            // Level 1: Medium detail (50%)
            lods[1] = new LOD(0.5f, lodGroup.GetLODs()[0].renderers);
            // Level 2: Low detail (25%)
            lods[2] = new LOD(0.25f, lodGroup.GetLODs()[0].renderers);

            lodGroup.SetLODs(lods);
            lodGroup.RecalculateBounds();
        }
    }
}
```

## Troubleshooting Common Issues

### ROS2 Connection Issues
- **Connection refused**: Check if ROS2 bridge is running and ports match
- **Serialization errors**: Ensure message types match between Unity and ROS2
- **Performance issues**: Reduce publishing frequency or simplify messages

### Rendering Issues
- **Low frame rate**: Reduce visual quality settings or use LOD
- **Lighting artifacts**: Check material configurations and light settings
- **Texture problems**: Verify texture import settings

### Physics Issues
- **Joints not moving**: Check Articulation Body configurations
- **Unrealistic movement**: Adjust joint limits and drive parameters
- **Stability problems**: Tune physics solver parameters

## Summary

In this chapter, we've explored how to use Unity for creating high-fidelity visual simulations of humanoid robots. We covered setting up Unity for robotics applications, creating realistic robot models, configuring advanced rendering and lighting, and integrating with ROS2 for complete simulation workflows. Unity's powerful rendering capabilities make it an excellent choice for creating photorealistic robot simulations.

## Next Steps

- Set up Unity with the Robotics packages
- Create a simple humanoid robot model in Unity
- Implement ROS2 communication
- Experiment with Unity Perception for synthetic data generation
- Learn about advanced Unity features for robotics simulation