using UnityEngine;
using System;
using System.Collections;
using System.IO.Ports;

public class Serial : MonoBehaviour {

	public bool arduinoOn = true;

	// Brush tip facing, left: y = 0-180, right: y = 180-360
	// Brush hairs facing you, when tip on left: z=0, when tip on right z=180
	public Vector3 angles;

	public float brushing = 0;
	public float brushingAcc = 0;
	public float brushingStrength = 0;
	public float[] brushingStrengthValues;
	public int brushingStrengthIndex = 0;
	public static string strIn;
	public static string message;
	SerialPort sp = new SerialPort("/dev/tty.usbmodem3321", 115200);

	void Start () {
		brushingStrengthValues = new float[5];
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
				string[] values = strIn.Split('|');
				float qy = -float.Parse(values[0]);
				float qz = -float.Parse(values[1]);
				float qx = float.Parse(values[2]);
				float qw = -float.Parse(values[3]);
				brushing = float.Parse(values[4]);
				brushingAcc = float.Parse(values[5]);

				if (++brushingStrengthIndex > 4) brushingStrengthIndex = 0;
				brushingStrengthValues[brushingStrengthIndex] = Math.Abs(brushingAcc);
				brushingStrength = 0;
				foreach (int i in brushingStrengthValues) brushingStrength += i;
				brushingStrength /= 5;

				if (brushing > 0) {
					transform.position = new Vector3(0, 0, 0);
					transform.position = transform.forward * (brushingAcc * 0.005f);
					renderer.material.color = new Color(1, 0, 0);
				} else {
					transform.position = new Vector3(0, 0, 0);
					renderer.material.color = new Color(1, 1, 1);
				}
				transform.rotation = new Quaternion(qx, qy, qz, qw);
				angles = transform.rotation.eulerAngles;
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
