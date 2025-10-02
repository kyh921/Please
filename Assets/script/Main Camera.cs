using UnityEngine;

public class FollowCam : MonoBehaviour
{
    public Transform Target;
    public Vector3 offset = new Vector3(0, 5, -10);  // ��� ���� ��ġ
    public float followSpeed = 5f;                   // ���󰡴� �ӵ�
    public float lookSpeed = 7f;                     // �ü� ��ȯ �ӵ�

    void LateUpdate()
    {
        if (!Target) return;

        // ��ǥ ��ġ
        Vector3 desiredPos = Target.position + Target.TransformDirection(offset);
        transform.position = Vector3.Lerp(transform.position, desiredPos, Time.deltaTime * followSpeed);

        // ��ǥ �ٶ󺸱�
        Quaternion lookRot = Quaternion.LookRotation(Target.position - transform.position, Vector3.up);
        transform.rotation = Quaternion.Slerp(transform.rotation, lookRot, Time.deltaTime * lookSpeed);
    }
}
