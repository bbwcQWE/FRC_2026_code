/** 视觉常量 - 配置视觉子系统的参数 */
package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

/** 视觉常量 - 配置视觉子系统的参数 */
public class VisionConstants {

  /** AprilTag场地类型枚举 */
  public enum AprilTagFieldType {
    kDefault("默认场地 (2026)"),
    kCustom("自定义场地 (11319)");

    private final String displayName;

    AprilTagFieldType(String displayName) {
      this.displayName = displayName;
    }

    public String getDisplayName() {
      return displayName;
    }
  }

  // AprilTag场地布局 - 动态加载
  public static AprilTagFieldLayout aprilTagLayout;

  // 场地选择器 - 用于在Dashboard上选择场地
  public static final SendableChooser<AprilTagFieldType> fieldChooser = new SendableChooser<>();

  // 当前选择的场地类型
  public static AprilTagFieldType currentFieldType = AprilTagFieldType.kDefault;

  static {
    // 配置默认选项
    fieldChooser.setDefaultOption(
        AprilTagFieldType.kDefault.getDisplayName(), AprilTagFieldType.kDefault);
    fieldChooser.addOption(AprilTagFieldType.kCustom.getDisplayName(), AprilTagFieldType.kCustom);

    // 初始化默认场地
    loadField(AprilTagFieldType.kDefault);

    // 注册到NetworkTable
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putData(
        "AprilTag Field Selector", fieldChooser);
  }

  /**
   * 加载指定的场地布局
   *
   * @param fieldType 场地类型
   */
  public static void loadField(AprilTagFieldType fieldType) {
    currentFieldType = fieldType;
    try {
      switch (fieldType) {
        case kCustom:
          // 加载自定义场地
          aprilTagLayout = AprilTagFieldLayout.loadFromResource("11319CostField_V1.json");
          System.out.println("[VisionConstants] 已加载自定义AprilTag场地: 11319CostField_V1.json");
          break;
        case kDefault:
        default:
          // 加载默认场地 (2025赛季)
          aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
          System.out.println("[VisionConstants] 已加载默认AprilTag场地: kDefaultField");
          break;
      }
    } catch (Exception e) {
      System.err.println("[VisionConstants] 加载场地失败: " + e.getMessage() + ", 回退到默认场地");
      try {
        aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        currentFieldType = AprilTagFieldType.kDefault;
      } catch (Exception e2) {
        System.err.println("[VisionConstants] 严重错误: 无法加载默认场地: " + e2.getMessage());
      }
    }
  }

  /** 根据Dashboard选择更新场地 - 应该在机器人启动时调用 */
  public static void updateFieldFromChooser() {
    AprilTagFieldType selected = fieldChooser.getSelected();
    if (selected != null && selected != currentFieldType) {
      loadField(selected);
    }
  }

  // ==================== PhotonVision 相机配置 ====================
  // 相机名称，必须与PhotonVision上配置的名称匹配
  public static String camera0Name = "camera_0";
  public static String camera1Name = "camera_1";

  // 机器人到相机的变换
  // X: 机器人前方, Y: 机器人左侧, Z: 机器人上方
  // 根据实际安装位置调整
  public static Transform3d robotToCamera0 =
      new Transform3d(0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, 0.0));
  public static Transform3d robotToCamera1 =
      new Transform3d(-0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, Math.PI));

  // 基本过滤阈值
  public static double maxAmbiguity = 0.3; // 最大歧义度
  public static double maxZError = 0.75; // 最大Z轴误差

  // 标准差基线（1米距离和1个标签）
  // （根据距离和标签数量自动调整）
  public static double linearStdDevBaseline = 0.02; // 米
  public static double angularStdDevBaseline = 0.06; // 弧度

  // 每个相机的标准差乘数
  // （调整以信任某些相机多于其他相机）
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // 相机 0
        1.0 // 相机 1
      };

  // PhotonVision观测的乘数 (PhotonVision使用单目/多目标姿态估计)
  public static double linearStdDevPhotonVisionFactor = 1.0;
  public static double angularStdDevPhotonVisionFactor = 1.0;

  // MegaTag 2观测的乘数 (保留用于Limelight兼容)
  public static double linearStdDevMegatag2Factor = 0.5; // 比完整3D解算更稳定
  public static double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY; // 无旋转数据
}
