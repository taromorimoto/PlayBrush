using UnityEngine;
using System.Collections;

public class EnemySpawner : MonoBehaviour {

	public int maxEnemyCount = 3;
	public int enemyCount = 0;

	// Use this for initialization
	void Start () {

	
	}
	
	// Update is called once per frame
	void Update () {
		GameObject[] gos = GameObject.FindGameObjectsWithTag("Respawn");

		if (gos.Length < maxEnemyCount) {
			GameObject enemy = (GameObject)Instantiate(Resources.Load("Enemy"));
			float x1 = 0;
			float x2 = 0;
			float y1 = 1;
			float y2 = 3;
			if (enemyCount++ > 20) {
				x1 = -10.0f;
				x2 = -5.0f;
				/*
				if (enemyCount < 30) {
					y1 = -10;
					y2 = -8;
				} else {
					y1 = 8;
					y2 = 10;
				}*/
				if (enemyCount > 40) enemyCount = 0;
			} else {
				x1 = 5.0f;
				x2 = 10.0f;
				/*
				if (enemyCount < 10) {
					y1 = -10;
					y2 = -8;
				} else {
					y1 = 8;
					y2 = 10;
				}*/
			}
			Vector3 pos = new Vector3(Random.Range(x1, x2), Random.Range(y1, y2), Random.Range(-10f, -8f));
			float scale = Random.Range(0, 2f);
			enemy.transform.position = pos.normalized * 10;
			enemy.transform.localScale += new Vector3(scale, scale, scale);
		}
	
	}
}
