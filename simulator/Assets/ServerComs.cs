using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;


public class ServerComs : MonoBehaviour
{   
    public enum State
    {
        Idle,
        Manual,
        Autonomous,
        Auto
    }

    public Transform robot;
    public Rigidbody rb;
    public Camera camera;
    private TelemetryModel msg = new TelemetryModel();

    public bool WebSocketServerEnabled;
    public bool DebugLogEnabled;

    private bool clientConnected = false;
    private WsClient client;
    private string server;  // server address

    private int sentTelemCount = 0;
    private int receivedTelemCount = 0;
    private int lastTelemTime = 0;
    public int telemIntervalMs = 1000;

    [SerializeField] private string host = "localhost"; // Server IP address
    [SerializeField] private int port = 3000; // Server port

    private void Awake()
    {
        if (!WebSocketServerEnabled)
            return;

        robot = GameObject.Find("Car").GetComponent<Transform>();
        camera = GameObject.Find("Car Cam").GetComponent<Camera>();
        rb = robot.GetComponent<Rigidbody>();

        server = "ws://" + host + ":" + port;
        if (DebugLogEnabled)
            Debug.Log("using websocket setver " + server);
        if (client == null)
        {
            client = new WsClient(server);
            ConnectToServer();
        }
    }

    public async void ConnectToServer()
    {
        if (DebugLogEnabled)
            Debug.Log("Awaiting connecting to server");

        await client.Connect();
        clientConnected = true;

        if (DebugLogEnabled)
            Debug.Log("Connected to server");
    }

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        if (!WebSocketServerEnabled) { return; }

        // Check if server send new messages
        var cqueue = client.receiveQueue;
        string msg;
        while (cqueue.TryPeek(out msg))
        {
            // Parse newly received messages
            cqueue.TryDequeue(out msg);
            HandleMessage(msg);
        }

        if (Time.time * 1000 > lastTelemTime + telemIntervalMs)
        {
            SendTelemetry();
            lastTelemTime = (int)Time.time * 1000;
        }
    }

    /// <summary>
    /// Method which sends data through websocket
    /// </summary>
    /// <param name="message">Message.</param>
    public void SendTelemetry()
    {
        msg.msg_num = sentTelemCount++;
        msg.msg_type = "telemetry";
        msg.timestamp = DateTime.Now.ToString("MM/dd/yyy HH:mm:ss.") + DateTime.Now.Millisecond.ToString();
        msg.source = "sim";
        msg.state = "autonmous"; // TODO: add manual vs autonmous
        
        msg.camera = System.Text.Encoding.UTF8.GetString(GetScreenshot());
        // TODO: Depth map

        msg.pos_x = robot.position.x;
        msg.pos_y = robot.position.y;
        msg.pos_z = robot.position.z;

        msg.heading = robot.eulerAngles.y;
        msg.roll = robot.eulerAngles.x;
        msg.pitch = robot.eulerAngles.z;

        msg.linear_accel_x = (rb.velocity.x - msg.vel_x) / Time.fixedDeltaTime;
        msg.linear_accel_y = (rb.velocity.y - msg.vel_y) / Time.fixedDeltaTime;
        msg.linear_accel_z = (rb.velocity.z - msg.vel_z) / Time.fixedDeltaTime;

        msg.ang_vel_x = rb.angularVelocity.x;
        msg.ang_vel_y = rb.angularVelocity.y;
        msg.ang_vel_z = rb.angularVelocity.z;

        msg.vel_x = rb.velocity.x;
        msg.vel_y = rb.velocity.y;
        msg.vel_z = rb.velocity.z;

        if (DebugLogEnabled)
            Debug.Log("Sending message #" + sentTelemCount + ", msg: " + msg);

        client.Send(JsonUtility.ToJson(msg));
    }


    private void HandleMessage(string msg)
    {
        if (DebugLogEnabled)
            Debug.Log("Received message #" + receivedTelemCount++ + ", msg: " + msg);

        // Deserializing message from the server
        var message = JsonUtility.FromJson<TelemetryModel>(msg);

        // Picking correct method for message handling
        switch (message.msg_type)
        {
            case "command":
                {
                    Debug.Log("Apply Forces Here");
                    // TODO: Apply motor torques here
                    /*for (int i = 0; i < 6; i++)
                        forces[i] = message.raw_thrust[i];*/
                }
                break;
            case "telemetry":
                {
                    // Debug.Log("Updating UI with telemetry from server");
                    // TODO: Sim robot mimics real robot, this might be better done through a zed api
                }
                break;
            case "reset":
                SceneManager.LoadScene(SceneManager.GetActiveScene().name);
                break;
            default:
                Debug.Log("Unknown message type: " + message.msg_type);
                break;
        }
    }


    // Take a "screenshot" of a camera's Render Texture.
    private byte[] GetScreenshot()
    {
        Camera Cam = GameObject.Find("Car Cam").GetComponent<Camera>();
        Debug.Log(Cam);

        // The Render Texture in RenderTexture.active is the one that will be read by ReadPixels.
        RenderTexture currentRT = RenderTexture.active;
        RenderTexture.active = camera.targetTexture; // TODO: Get Depth here?

        // Render the camera's view.
        camera.Render();

        // Make a new texture and read the active Render Texture into it.
        Debug.Log(camera);  
        Texture2D image = new Texture2D(camera.targetTexture.width, camera.targetTexture.height);
        image.ReadPixels(new Rect(0, 0, camera.targetTexture.width, camera.targetTexture.height), 0, 0);
        image.Apply();

        // Replace the original active Render Texture.
        RenderTexture.active = currentRT;
        return image.EncodeToJPG();
    }
}
