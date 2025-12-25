package com.team5959;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

/** Add your docs here. */
public class Vision {
    private final PhotonCamera leftFrontCamera;
    private final PhotonCamera rightFrontCamera;
    private final PhotonCamera objectCamera;

    private final PhotonPoseEstimator leftPoseEstimator;
    private final PhotonPoseEstimator rightPoseEstimator;
    private AprilTagFieldLayout aprilTagFieldLayout;

    // --- VARIABLES DE SIMULACIÓN ---
    private VisionSystemSim visionSim;
    private PhotonCameraSim leftCameraSim;
    private PhotonCameraSim rightCameraSim;

    // Constantes de Confianza (StdDevs)
    // X, Y, Rotacion. (Rotacion infinita para confiar siempre en el Gyro)
    
    private static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    private static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

    private static final Transform3d LEFT_FRONT_ROBOT_TO_CAM = new Transform3d(
            new Translation3d(0.3, 0.3, 0.2),
            new Rotation3d(0, Units.degreesToRadians(22), Units.degreesToRadians(45)));

    private static final Transform3d RIGHT_FRONT_ROBOT_TO_CAM = new Transform3d(
            new Translation3d(0.3, -0.3, 0.2),
            new Rotation3d(0, Units.degreesToRadians(22), Units.degreesToRadians(-45)));

    public Vision() {
        leftFrontCamera = new PhotonCamera("leftFrontCamera");
        rightFrontCamera = new PhotonCamera("rightFrontCamera");
        objectCamera = new PhotonCamera("cameraObjects");

        try {
            // Usamos 2024 Crescendo ya que tu librería aún no tiene 2025
            aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
        } catch (Exception e) {
            e.printStackTrace();
            // Inicializar un layout vacío para evitar NullPointerException si falla la carga
            aprilTagFieldLayout = new AprilTagFieldLayout(new ArrayList<>(), 16.54, 8.21);
        }

        // Inicializar Estimador Izquierdo
        leftPoseEstimator = new PhotonPoseEstimator(
                aprilTagFieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                LEFT_FRONT_ROBOT_TO_CAM);

        // Inicializar Estimador Derecho
        rightPoseEstimator = new PhotonPoseEstimator(
                aprilTagFieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                RIGHT_FRONT_ROBOT_TO_CAM);

        // Configuración extra: Usar MultiTag es lo más preciso.
        // Si falla (ve 1 solo tag), usa la estrategia de fallback (Lowest Ambiguity).
        leftPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        rightPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        // --- INICIALIZACIÓN DE SIMULACIÓN ---
        // Esto solo se ejecuta si estamos simulando en la PC
        if (RobotBase.isSimulation()) {
            // 1. Crear el sistema de visión simulado
            visionSim = new VisionSystemSim("main");
            
            // 2. Agregar los AprilTags del campo al simulador
            visionSim.addAprilTags(aprilTagFieldLayout);

            // 3. Definir propiedades de la cámara simulada (OV9281 aprox)
            SimCameraProperties cameraProps = new SimCameraProperties();
            cameraProps.setCalibration(640, 480, Rotation2d.fromDegrees(70)); // FOV aprox 70 grados
            cameraProps.setCalibError(0.25, 0.10); // Simular ruido/error
            cameraProps.setFPS(30);
            cameraProps.setAvgLatencyMs(30);
            cameraProps.setLatencyStdDevMs(5);

            // 4. Crear los simuladores de cámara vinculados a las cámaras reales
            leftCameraSim = new PhotonCameraSim(leftFrontCamera, cameraProps);
            rightCameraSim = new PhotonCameraSim(rightFrontCamera, cameraProps);

            // 5. Agregarlas al mundo simulado con su posición respecto al robot
            visionSim.addCamera(leftCameraSim, LEFT_FRONT_ROBOT_TO_CAM);
            visionSim.addCamera(rightCameraSim, RIGHT_FRONT_ROBOT_TO_CAM);
            
            // Habilitar visualización de wireframes para ver los tags en el stream
            leftCameraSim.enableDrawWireframe(true);
            rightCameraSim.enableDrawWireframe(true);
        }
    }

    /**
     * Obtiene las estimaciones de pose de todas las cámaras de AprilTags visibles.
     * @return Lista de estimaciones para inyectar en el SwerveDrivePoseEstimator.
     */
    public List<EstimatedRobotPose> getEstimatedGlobalPoses() {
        List<EstimatedRobotPose> estimates = new ArrayList<>();
        
        if (aprilTagFieldLayout == null) return estimates; // Seguridad por si falla carga del mapa

        // 1. Procesar Cámara Izquierda
        for(PhotonPipelineResult leftResult : leftFrontCamera.getAllUnreadResults()) {
        // Solo intentamos actualizar si el resultado tiene datos válidos
        if (leftResult.hasTargets()) {
             Optional<EstimatedRobotPose> leftEst = leftPoseEstimator.update(leftResult);
             leftEst.ifPresent(estimates::add);
            }
        }
        // 2. Procesar Cámara Derecha
        for (PhotonPipelineResult rightResult : rightFrontCamera.getAllUnreadResults()) {
        if (rightResult.hasTargets()) {
            Optional<EstimatedRobotPose> rightEst = rightPoseEstimator.update(rightResult);
            rightEst.ifPresent(estimates::add);
            }
        }
        return estimates;
    }

    /**
     * Calcula la Matriz de Desviación Estándar (Confianza) para una estimación de visión.
     * La lógica es: A mayor distancia promedio de los tags, menor confianza (números más grandes).
     */
    public Matrix<N3, N1> getEstimationStdDevs(EstimatedRobotPose estimatedPose) {
        var estStdDevs = kSingleTagStdDevs; // Default: confianza media en X/Y, nula en rotación
        var targets = estimatedPose.targetsUsed;
        int numTags = 0;
        double avgDist = 0;

        // Calcular distancia promedio a los tags visibles
        for (var target : targets) {
            var tagPose = aprilTagFieldLayout.getTagPose(target.getFiducialId());
            if (tagPose.isEmpty()) continue;
            numTags++;
            // Usamos la distancia desde la cámara al tag
            avgDist += target.getBestCameraToTarget().getTranslation().getNorm();
        }

        if (numTags == 0) return estStdDevs;
        avgDist /= numTags;

        // FÓRMULA DE CONFIANZA:
        // Si hay más de 1 tag, confiamos mucho (xyStdDev bajo).
        // Si hay 1 solo tag, la confianza empeora linealmente con la distancia.
        if (numTags > 1) {
            // Si vemos múltiples tags, usamos la base de confianza alta
             estStdDevs = kMultiTagStdDevs
        }

        // Si solo vemos 1 tag y está lejos (> 4m), lo ignoramos (confianza infinita = ignorar)
        if (numTags == 1 && avgDist > 4) {
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        } else {
            // Escalamos la confianza basada en la distancia.
            // Factor: 1 + (distancia^2 / 30). Curva suave.
            // Ejemplo: a 3m -> 1 + (9/30) = 1.3x de error.
            estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
        }
        
        return estStdDevs;
    }

    /**
     * Obtiene los datos crudos de la cámara de objetos (OV9782).
     * Úsalo en tus comandos para auto-alineación con Notas/Corales.
     * @return Resultado del pipeline (contiene yaw, pitch, area, targets).
     */
    public PhotonPipelineResult getObjectCameraResult() {
        return objectCamera.getLatestResult();
    }

    /**
     * Llama a esto periódicamente SOLO en simulación para actualizar 
     * dónde "cree" el simulador que está el robot.
     */
    public void simulationPeriodic(Pose2d robotSimPose) {
        if (visionSim != null) {
            visionSim.update(robotSimPose);
        }
    }
}
