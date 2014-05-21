using UnityEngine;

using System.Collections;

using System.IO;

using System.Text;

public class AudioTest : MonoBehaviour {
	private Serial serial;

	void Start () {
		serial = GetComponent<Serial>();

		KalimbaPd.Init();
		
		//KalimbaPd.OpenFile("kalimbaTest.pd", "pd");
		KalimbaPd.OpenFile("granular.pd", "pd");
	}

	void Update () {
		Vector3 angles = serial.angles;
		if (serial.brushing > 0) {
			//KalimbaPd.SendFloat(1, "brushing");
			KalimbaPd.SendFloat(serial.brushingAcc, "brushingAcc");
			KalimbaPd.SendFloat(serial.brushingStrength, "brushingStrength");
			KalimbaPd.SendFloat(angles.x, "x");
			KalimbaPd.SendFloat(angles.y, "y");
			KalimbaPd.SendFloat(angles.z, "z");
		} else {
			KalimbaPd.SendFloat(0, "brushing");
		}
		//print ("x:" + angles.x + " y:" + angles.y + " z:" + angles.z);
	}

	public void enemyDeath(Enemy enemy) {
		KalimbaPd.SendFloat(enemy.transform.localScale.x, "enemyDeathScale");
		KalimbaPd.SendBangToReceiver("enemyDeath");
	}
}
