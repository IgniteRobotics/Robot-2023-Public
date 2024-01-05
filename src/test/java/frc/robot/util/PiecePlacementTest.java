package frc.robot.util;

import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.util.PiecePlacement.Location.*;

public class PiecePlacementTest {
    @Test
    public void testIsCone() {
        assertTrue(PiecePlacement.isConePosition(27));
        assertTrue(PiecePlacement.isConePosition(33));
        assertTrue(PiecePlacement.isConePosition(36));
        assertTrue(PiecePlacement.isConePosition(42));
        assertTrue(PiecePlacement.isConePosition(45));
        assertTrue(PiecePlacement.isConePosition(51));

        assertTrue(PiecePlacement.isConePosition(28));
        assertTrue(PiecePlacement.isConePosition(34));
        assertTrue(PiecePlacement.isConePosition(37));
        assertTrue(PiecePlacement.isConePosition(43));
        assertTrue(PiecePlacement.isConePosition(46));
        assertTrue(PiecePlacement.isConePosition(52));

        assertTrue(PiecePlacement.isConePosition(0));
        assertTrue(PiecePlacement.isConePosition(6));
        assertTrue(PiecePlacement.isConePosition(9));
        assertTrue(PiecePlacement.isConePosition(15));
        assertTrue(PiecePlacement.isConePosition(18));
        assertTrue(PiecePlacement.isConePosition(24));

        assertTrue(PiecePlacement.isConePosition(1));
        assertTrue(PiecePlacement.isConePosition(7));
        assertTrue(PiecePlacement.isConePosition(10));
        assertTrue(PiecePlacement.isConePosition(16));
        assertTrue(PiecePlacement.isConePosition(19));
        assertTrue(PiecePlacement.isConePosition(25));

        assertFalse(PiecePlacement.isConePosition(30));
        assertFalse(PiecePlacement.isConePosition(39));
        assertFalse(PiecePlacement.isConePosition(48));

        assertFalse(PiecePlacement.isConePosition(31));
        assertFalse(PiecePlacement.isConePosition(40));
        assertFalse(PiecePlacement.isConePosition(49));

        assertFalse(PiecePlacement.isConePosition(3));
        assertFalse(PiecePlacement.isConePosition(12));
        assertFalse(PiecePlacement.isConePosition(21));

        assertFalse(PiecePlacement.isConePosition(4));
        assertFalse(PiecePlacement.isConePosition(13));
        assertFalse(PiecePlacement.isConePosition(22));
    }

    @Test
    public void testSerialization() {
        Translation2d[] defaultArr = PiecePlacement.getDefaultPositions();
        
        String s = PiecePlacement.serialize(defaultArr);
        
        Translation2d[] result = PiecePlacement.deserialize(s);

        assertArrayEquals(defaultArr, result);
    }
    @Test
    public void getPlacementIndexIsCorrect() {
        // Blue Alliance
        assertEquals(2, PiecePlacement.getPlacementIndex(Alliance.Blue, Grid.G1, Position.LEFT, Height.LOW));
        assertEquals(1, PiecePlacement.getPlacementIndex(Alliance.Blue, Grid.G1, Position.LEFT, Height.MIDDLE));
        assertEquals(0, PiecePlacement.getPlacementIndex(Alliance.Blue, Grid.G1, Position.LEFT, Height.HIGH));
        assertEquals(5, PiecePlacement.getPlacementIndex(Alliance.Blue, Grid.G1, Position.CENTER, Height.LOW));
        assertEquals(4, PiecePlacement.getPlacementIndex(Alliance.Blue, Grid.G1, Position.CENTER, Height.MIDDLE));
        assertEquals(3, PiecePlacement.getPlacementIndex(Alliance.Blue, Grid.G1, Position.CENTER, Height.HIGH));
        assertEquals(8, PiecePlacement.getPlacementIndex(Alliance.Blue, Grid.G1, Position.RIGHT, Height.LOW));
        assertEquals(7, PiecePlacement.getPlacementIndex(Alliance.Blue, Grid.G1, Position.RIGHT, Height.MIDDLE));
        assertEquals(6, PiecePlacement.getPlacementIndex(Alliance.Blue, Grid.G1, Position.RIGHT, Height.HIGH));
    
        assertEquals(11, PiecePlacement.getPlacementIndex(Alliance.Blue, Grid.G2, Position.LEFT, Height.LOW));
        assertEquals(10, PiecePlacement.getPlacementIndex(Alliance.Blue, Grid.G2, Position.LEFT, Height.MIDDLE));
        assertEquals(9, PiecePlacement.getPlacementIndex(Alliance.Blue, Grid.G2, Position.LEFT, Height.HIGH));
        assertEquals(14, PiecePlacement.getPlacementIndex(Alliance.Blue, Grid.G2, Position.CENTER, Height.LOW));
        assertEquals(13, PiecePlacement.getPlacementIndex(Alliance.Blue, Grid.G2, Position.CENTER, Height.MIDDLE));
        assertEquals(12, PiecePlacement.getPlacementIndex(Alliance.Blue, Grid.G2, Position.CENTER, Height.HIGH));
        assertEquals(17, PiecePlacement.getPlacementIndex(Alliance.Blue, Grid.G2, Position.RIGHT, Height.LOW));
        assertEquals(16, PiecePlacement.getPlacementIndex(Alliance.Blue, Grid.G2, Position.RIGHT, Height.MIDDLE));
        assertEquals(15, PiecePlacement.getPlacementIndex(Alliance.Blue, Grid.G2, Position.RIGHT, Height.HIGH));

        assertEquals(20, PiecePlacement.getPlacementIndex(Alliance.Blue, Grid.G3, Position.LEFT, Height.LOW));
        assertEquals(19, PiecePlacement.getPlacementIndex(Alliance.Blue, Grid.G3, Position.LEFT, Height.MIDDLE));
        assertEquals(18, PiecePlacement.getPlacementIndex(Alliance.Blue, Grid.G3, Position.LEFT, Height.HIGH));
        assertEquals(23, PiecePlacement.getPlacementIndex(Alliance.Blue, Grid.G3, Position.CENTER, Height.LOW));
        assertEquals(22, PiecePlacement.getPlacementIndex(Alliance.Blue, Grid.G3, Position.CENTER, Height.MIDDLE));
        assertEquals(21, PiecePlacement.getPlacementIndex(Alliance.Blue, Grid.G3, Position.CENTER, Height.HIGH));
        assertEquals(26, PiecePlacement.getPlacementIndex(Alliance.Blue, Grid.G3, Position.RIGHT, Height.LOW));
        assertEquals(25, PiecePlacement.getPlacementIndex(Alliance.Blue, Grid.G3, Position.RIGHT, Height.MIDDLE));
        assertEquals(24, PiecePlacement.getPlacementIndex(Alliance.Blue, Grid.G3, Position.RIGHT, Height.HIGH));
    
        // Red Alliance
        assertEquals(29, PiecePlacement.getPlacementIndex(Alliance.Red, Grid.G1, Position.LEFT, Height.LOW));
        assertEquals(28, PiecePlacement.getPlacementIndex(Alliance.Red, Grid.G1, Position.LEFT, Height.MIDDLE));
        assertEquals(27, PiecePlacement.getPlacementIndex(Alliance.Red, Grid.G1, Position.LEFT, Height.HIGH));
        assertEquals(32, PiecePlacement.getPlacementIndex(Alliance.Red, Grid.G1, Position.CENTER, Height.LOW));
        assertEquals(31, PiecePlacement.getPlacementIndex(Alliance.Red, Grid.G1, Position.CENTER, Height.MIDDLE));
        assertEquals(30, PiecePlacement.getPlacementIndex(Alliance.Red, Grid.G1, Position.CENTER, Height.HIGH));
        assertEquals(35, PiecePlacement.getPlacementIndex(Alliance.Red, Grid.G1, Position.RIGHT, Height.LOW));
        assertEquals(34, PiecePlacement.getPlacementIndex(Alliance.Red, Grid.G1, Position.RIGHT, Height.MIDDLE));
        assertEquals(33, PiecePlacement.getPlacementIndex(Alliance.Red, Grid.G1, Position.RIGHT, Height.HIGH));
    
        assertEquals(38, PiecePlacement.getPlacementIndex(Alliance.Red, Grid.G2, Position.LEFT, Height.LOW));
        assertEquals(37, PiecePlacement.getPlacementIndex(Alliance.Red, Grid.G2, Position.LEFT, Height.MIDDLE));
        assertEquals(36, PiecePlacement.getPlacementIndex(Alliance.Red, Grid.G2, Position.LEFT, Height.HIGH));
        assertEquals(41, PiecePlacement.getPlacementIndex(Alliance.Red, Grid.G2, Position.CENTER, Height.LOW));
        assertEquals(40, PiecePlacement.getPlacementIndex(Alliance.Red, Grid.G2, Position.CENTER, Height.MIDDLE));
        assertEquals(39, PiecePlacement.getPlacementIndex(Alliance.Red, Grid.G2, Position.CENTER, Height.HIGH));
        assertEquals(44, PiecePlacement.getPlacementIndex(Alliance.Red, Grid.G2, Position.RIGHT, Height.LOW));
        assertEquals(43, PiecePlacement.getPlacementIndex(Alliance.Red, Grid.G2, Position.RIGHT, Height.MIDDLE));
        assertEquals(42, PiecePlacement.getPlacementIndex(Alliance.Red, Grid.G2, Position.RIGHT, Height.HIGH));

        assertEquals(47, PiecePlacement.getPlacementIndex(Alliance.Red, Grid.G3, Position.LEFT, Height.LOW));
        assertEquals(46, PiecePlacement.getPlacementIndex(Alliance.Red, Grid.G3, Position.LEFT, Height.MIDDLE));
        assertEquals(45, PiecePlacement.getPlacementIndex(Alliance.Red, Grid.G3, Position.LEFT, Height.HIGH));
        assertEquals(50, PiecePlacement.getPlacementIndex(Alliance.Red, Grid.G3, Position.CENTER, Height.LOW));
        assertEquals(49, PiecePlacement.getPlacementIndex(Alliance.Red, Grid.G3, Position.CENTER, Height.MIDDLE));
        assertEquals(48, PiecePlacement.getPlacementIndex(Alliance.Red, Grid.G3, Position.CENTER, Height.HIGH));
        assertEquals(53, PiecePlacement.getPlacementIndex(Alliance.Red, Grid.G3, Position.RIGHT, Height.LOW));
        assertEquals(52, PiecePlacement.getPlacementIndex(Alliance.Red, Grid.G3, Position.RIGHT, Height.MIDDLE));
        assertEquals(51, PiecePlacement.getPlacementIndex(Alliance.Red, Grid.G3, Position.RIGHT, Height.HIGH));
    }
}
