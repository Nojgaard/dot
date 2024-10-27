from scipy.spatial.transform import Rotation


def quat_to_euler(quat):
    return Rotation.from_quat(quat, scalar_first=True).as_euler("XYZ")
