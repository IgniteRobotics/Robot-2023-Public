package frc.robot.util;

import java.nio.ByteBuffer;
import java.nio.charset.StandardCharsets;

import javax.tools.DocumentationTool.Location;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.util.PiecePlacement.Location.Grid;

public class PiecePlacement {
    public static final double NEW_POINT_WEIGHT = 0.3;

    /**
     * This array represents all 54 placement positions on the field for both the blue and red side.
     * 
     *                               RED SIDE
     * |     Blue    | 27  30  33 | 36  39  42 | 45  48  51 |
     * | Substations | 28  31  34 | 37  40  43 | 46  49  52 |
     * |             | 29  32  35 | 38  41  44 | 47  50  53 |
     * |             |                                      |
     * |             |                                      |
     * |             |                                      |
     * |                                                    |
     * |                                                    |
     * |                                                    |
     * |                                                    |
     * |             |                                      |
     * |             |                                      |
     * |             |                                      |
     * |             |  2   5   8 | 11  14  17 | 20  23  26 |
     * |     Red     |  1   4   7 | 10  13  16 | 19  22  25 |
     * | Substations |  0   3   6 |  9  12  15 | 18  21  24 |
     *                               BLUE SIDE
     * 
     */
    public Translation2d[] placementPositions = new Translation2d[54];

    private DataLog log = DataLogManager.getLog();
    private DoubleArrayLogEntry actualPlacementLog = new DoubleArrayLogEntry(log, "/PiecePlacement/actual");
    private DoubleArrayLogEntry predictedPlacementLog = new DoubleArrayLogEntry(log, "/PiecePlacement/predicted");

    private static PiecePlacement instance;

    public static PiecePlacement getInstance() {
        if(instance == null) {
            instance = new PiecePlacement();
        }

        return instance;
    }

    private PiecePlacement() {
        
    }

    public void initialize() {
        Preferences.initString("PiecePlacement/data", serialize(getDefaultPositions()));
        load();
    }

    public Translation2d getPositionMeasurement(Alliance alliance, Location.Grid grid, Location.Position position, Location.Height height) {
        return this.placementPositions[getPlacementIndex(alliance, grid, position, height)];
    }

    public void addPositionMeasurement(Translation2d placePosition, Alliance alliance, Location.Grid grid, Location.Position position, Location.Height height) {
        int index = getPlacementIndex(alliance, grid, position, height);
        Translation2d old = placementPositions[index];

        actualPlacementLog.append(new double[] { placePosition.getX(), placePosition.getY(), index });
        predictedPlacementLog.append(new double[] { old.getX(), old.getY(), index });

        // commenting out this code because it has issues
        // double newX = (1 - NEW_POINT_WEIGHT) * old.getX() + NEW_POINT_WEIGHT * placePosition.getX();
        // double newY = (1 - NEW_POINT_WEIGHT) * old.getY() + NEW_POINT_WEIGHT * placePosition.getY();

        // Translation2d newTranslation = new Translation2d(newX, newY);
        // placementPositions[index] = newTranslation;

        // if(isConePosition(index)) {
        //     updateConeXInRow(index, newX);
        // } else {
        //     updateCubeXInRow(index, newX);
        // }

        // updateYInColumn(index, newY);

        // save();
    }

    public static boolean isConePosition(int index) {
        int mod = ((index - (index % 3)) / 3) % 3;
        return mod == 0 || mod == 2;
    }

    private void updateYInColumn(int index, double yOffset) {
        // int column = index - (index % 3);
        // for(int i = 0; i < 3; ++i) {
        //     Translation2d pos = placementPositions[column + i];
        //     placementPositions[column + i] = new Translation2d(pos.getX(), yOffset);
        // }
    }

    private void updateConeXInRow(int index, double depth) {
        int row = index % 3;
        
        if(index > 26) {
            row = 27 + row;
        }

        // we use the same x position for hybrid nodes
        if(row == 2 || row == 29) {
            for(int i = 0; i < 9; ++i) {
                Translation2d pos = placementPositions[row + i * 3];
                placementPositions[row + i * 3] = new Translation2d(depth, pos.getY());
            }
        } else {
            for(int i = 0; i < 9; ++i) {
                if(i % 3 == 0 || i % 3 == 2) {
                    Translation2d pos = placementPositions[row + i * 3];
                    placementPositions[row + i * 3] = new Translation2d(depth, pos.getY());
                }
            }
        }
    }

    private void updateCubeXInRow(int index, double depth) {
        int row = index % 3;
        
        if(index > 26) {
            row = 27 + row;
        }

        // we use the same x position for hybrid nodes
        if(row == 2 || row == 29) {
            for(int i = 0; i < 9; ++i) {
                Translation2d pos = placementPositions[row + i * 3];
                placementPositions[row + i * 3] = new Translation2d(depth, pos.getY());
            }
        } else {
            for(int i = 0; i < 9; ++i) {
                if(i % 3 == 1) {
                    Translation2d pos = placementPositions[row + i * 3];
                    placementPositions[row + i * 3] = new Translation2d(depth, pos.getY());
                }
            }
        }
    }

    public void save() {
        Preferences.setString("PiecePlacement/data", serialize(placementPositions));
    }

    public void load() {
        Translation2d[] result = deserialize(Preferences.getString("PiecePlacement/data", ""));
        if(result.length == 0) {
            result = getDefaultPositions();
            DriverStation.reportError("Failed to deserialize PiecePlacement data from Preferences. Reverting to default positions.", false);
        }

        this.placementPositions = result;
    }

    public static String serialize(Translation2d[] positions) {
        return saveToString(serializePositions(positions));
    }

    public static Translation2d[] deserialize(String s) {
        return deserializePositions(loadFromString(s));
    }

    private static double[] loadFromString(String s) {
        byte[] bytes = s.getBytes(StandardCharsets.UTF_16LE);
        ByteBuffer bb = ByteBuffer.wrap(bytes);

        double[] arr = new double[bytes.length / 8];
        for(int i = 0 ; i < arr.length; ++i) {
            arr[i] = bb.getDouble();
        }

        return arr;
    }

    private static String saveToString(double[] dd) {
        ByteBuffer bb = ByteBuffer.allocate(dd.length * 8);
        for(double d : dd) {
            bb.putDouble(d);
        }

        return new String(bb.array(), StandardCharsets.UTF_16LE);
    }

    private static double[] serializePositions(Translation2d[] positions) {
        double[] arr = new double[positions.length * 2];
        for(int i = 0; i < positions.length; ++i) {
            arr[i * 2] = Math.max(positions[i].getX(), 0);
            arr[i * 2 + 1] = Math.max(positions[i].getY(), 0);
        }

        return arr;
    }

    private static Translation2d[] deserializePositions(double[] arr) {
        Translation2d[] positions = new Translation2d[arr.length / 2];
        for(int i = 0; i < positions.length; ++i) {
            positions[i] = new Translation2d(arr[i * 2], arr[i * 2 + 1]);
        }

        return positions;
    }

    public static Translation2d[] getDefaultPositions() {
        Translation2d[] defaultPositions = new Translation2d[54];
        double[] yPositions = { 0.5, 1.071626, 1.75, 2.15, 2.748026, 3.3, 3.8, 4.424426, 5.0 };
        double[] xPositions = { 1.85, 2.1, 2, 14.5, 14.45, 14.65 };

        for(int k = 0; k < 3; ++k) {
            for(int i = 0; i < 9; ++i) {
                defaultPositions[k + i * 3] = new Translation2d(xPositions[k], yPositions[8 - i]);
            }
        }

        for(int k = 0; k < 3; ++k) {
            for(int i = 0; i < 9; ++i) {
                defaultPositions[29 - k + i * 3] = new Translation2d(xPositions[k + 3], yPositions[8 - i]);
            }
        }

        return defaultPositions;
    }

    public static int getPlacementIndex(Alliance alliance, Location.Grid grid, Location.Position position, Location.Height height) {
        int n = grid.n * 9 + position.n * 3 + height.n;
        
        if(alliance == Alliance.Red) {
            n += 27;
        }

        return n;
    }

    public static Location.Position getPositionFromJoystick(Joystick joystick, Alliance alliance) {
        if(alliance == Alliance.Blue) {
            if(joystick.getRawButton(XboxController.Button.kX.value)) {
                return Location.Position.LEFT;
            } else if(joystick.getRawButton(XboxController.Button.kY.value)) {
                return Location.Position.CENTER;
            } else if(joystick.getRawButton(XboxController.Button.kB.value)) {
                return Location.Position.RIGHT;
            }
        } else {
            if(joystick.getRawButton(XboxController.Button.kX.value)) {
                return Location.Position.RIGHT;
            } else if(joystick.getRawButton(XboxController.Button.kY.value)) {
                return Location.Position.CENTER;
            } else if(joystick.getRawButton(XboxController.Button.kB.value)) {
                return Location.Position.LEFT;
            }
        }

        return null;
    }

    public static Location.Height getHeightFromJoystick(Joystick joystick, Alliance alliance) {
        if(joystick.getRawButton(XboxController.Button.kY.value)) {
            return Location.Height.HIGH;
        } else if(joystick.getRawButton(XboxController.Button.kB.value)) {
            return Location.Height.MIDDLE;
        } else if(joystick.getRawButton(XboxController.Button.kA.value)) {
            return Location.Height.LOW;
        }

        return null;
    }

    public static Location.Grid getClosestGrid(DriveSubsystem driveSubsystem) {
        double robotYPosition = driveSubsystem.getPose().getY();
        double grid1April = 4.424426;
        double grid2April = 2.748026;
        double grid3April = 1.071626;

        double d1 = Math.abs(robotYPosition - grid1April);
        double d2 = Math.abs(robotYPosition - grid2April);
        double d3 = Math.abs(robotYPosition - grid3April);

        if(d1 < d2 && d1 < d3) {
            return Location.Grid.G1;
        } else if(d2 < d1 && d2 < d3) {
            return Location.Grid.G2;
        } else {
            return Location.Grid.G3;
        }
    }

    public static class Location {
        public static enum Grid {
            G1(0), G2(1), G3(2);

            public final int n;

            private Grid(int n) {
                this.n = n;
            }
        }

        public static enum Position {
            LEFT(0), CENTER(1), RIGHT(2);

            public final int n;

            private Position(int n) {
                this.n = n;
            }
        }

        public static enum Height {
            LOW(2), MIDDLE(1), HIGH(0);

            public final int n;

            private Height(int n) {
                this.n = n;
            }
        }
    }
}
