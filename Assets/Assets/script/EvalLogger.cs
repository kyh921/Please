using System.IO;
using UnityEngine;

public class EvalLogger : MonoBehaviour
{
    public DroneTeamManager team;           // 팀 매니저 연결
    public string fileName = "eval_qoe_cov_ene.csv";

    private StreamWriter writer;
    private int stepIndex = 0;

    void Start()
    {
        if (team == null)
        {
            Debug.LogError("[EvalLogger] team이 설정되지 않았습니다.");
            enabled = false;
            return;
        }

        string dir = Path.Combine(Application.persistentDataPath, "Logs");
        Directory.CreateDirectory(dir);
        string path = Path.Combine(dir, fileName);

        writer = new StreamWriter(path, false, System.Text.Encoding.UTF8);
        writer.WriteLine("step,qoe_avg,cov_avg,ene_avg");

        Debug.Log("[EvalLogger] Save path: " + path);
    }

    void OnDestroy()
    {
        if (writer != null)
        {
            writer.Flush();
            writer.Close();
        }
    }

    void FixedUpdate()
    {
        if (writer == null || team == null || team.agents == null || team.agents.Count == 0)
            return;

        int alive = 0;
        float qoe = 0f, cov = 0f, ene = 0f;

        foreach (var a in team.agents)
        {
            if (a == null || a.IsEliminated) continue;

            alive++;
            qoe += a.debugQoE;
            cov += a.debugCov;
            ene += a.debugEne;
        }

        if (alive == 0)
        {
            Debug.Log("[EvalLogger] All agents eliminated. Stop logging.");
            enabled = false;
            return;
        }

        qoe /= alive;
        cov /= alive;
        ene /= alive;

        writer.WriteLine($"{stepIndex},{qoe},{cov},{ene}");
        stepIndex++;
    }
}
