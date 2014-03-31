using UnityEngine;
using System.Collections;
using System.IO.Ports;

public class Serial : MonoBehaviour {

	public bool arduinoOn = true;
	public static string strIn;
	public static string message;
	SerialPort sp = new SerialPort("/dev/tty.usbmodem1421", 115200);

	void Start () {
		if (arduinoOn) {
			OpenConnection();
		}
	}

	private static float to360(float val) {
		if (val < 0) return val + 360;
		if (val > 360) return val - 360;
		return val;
	}

	void Update () {
		if (arduinoOn) {
			if (sp.IsOpen) {
				sp.ReadTimeout = 1000;
				strIn = sp.ReadLine();
				//print("read:" + strIn);
				string[] values = strIn.Split('|');
				float y = -float.Parse(values[0]);
				float z = -float.Parse(values[1]);
				float x = float.Parse(values[2]);
				float w = -float.Parse(values[3]);
				float brushing = float.Parse(values[4]);
				float brushingAcc = float.Parse(values[5]);

				if (brushing > 0) {
					transform.position = new Vector3(0, 0, 0);
					transform.position = transform.forward * (brushingAcc * 0.005f);
					renderer.material.color = new Color(1, 0, 0);
				} else {
					transform.position = new Vector3(0, 0, 0);
					renderer.material.color = new Color(1, 1, 1);
				}
				transform.rotation = new Quaternion(x, y, z, w);
				//print("pos:" + transform.position.ToString() + " localpos:" + transform.localPosition.ToString() + " forward:" + transform.forward);
			}
		}
	}

	public void OpenConnection() {
		if (sp != null) {
			if (sp.IsOpen) {
				sp.Close();
				message = "Closing port, because it was already open!";
			} else {
				sp.Open();  // opens the connection
				sp.ReadTimeout = 50;  // sets the timeout value before reporting error
				message = "Port Opened!";
			}
		} else {
			if (sp.IsOpen) {
				print("Port is already open");
			} else {
				print("Port == null");
			}
		}
	}
	
	void OnApplicationQuit() {
		sp.Close();
	}

}
