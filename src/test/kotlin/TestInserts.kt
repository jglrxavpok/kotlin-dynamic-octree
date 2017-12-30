import org.jglrxavpok.octree.AABB
import org.jglrxavpok.octree.Octree
import org.joml.Vector3f
import org.junit.Test
import org.junit.Assert.*
import kotlin.concurrent.fixedRateTimer

class TestInserts {

    @Test
    fun splitIfMaxThresholdOvercome() {
        val root = Octree<AABB>(AABB(Vector3f(-1f), Vector3f(1f)), parent = null)
        root.minObjectThreshold = 1
        root.maxObjectThreshold = 2
        assertTrue("Root should be leaf right now", root.isLeaf)

        root.add(AABB(Vector3f(0f), Vector3f(0.5f)))
        assertTrue("Root should still be leaf right now", root.isLeaf)

        root.add(AABB(Vector3f(-0.5f), Vector3f(0f)))
        assertFalse("Root should have been split by now", root.isLeaf)
    }

    @Test
    fun splitAndMerge() {
        val root = Octree<AABB>(AABB(Vector3f(-1f), Vector3f(1f)), parent = null)
        root.minObjectThreshold = 1
        root.maxObjectThreshold = 2
        assertTrue("Root should be leaf right now", root.isLeaf)

        val firstBox = AABB(Vector3f(0f), Vector3f(0.5f))
        root.add(firstBox)
        assertTrue("Root should still be leaf right now", root.isLeaf)

        val secondBox = AABB(Vector3f(-0.5f), Vector3f(0f))
        root.add(secondBox)
        assertFalse("Root should have been split by now", root.isLeaf)

        root.deleteElement(firstBox)
        root.deleteElement(secondBox)
        assertTrue("Root should have been merged", root.isLeaf)
    }

    @Test
    fun intersectionTest() {
        val root = Octree<AABB>(AABB(Vector3f(-1f), Vector3f(1f)), parent = null)
        root.minObjectThreshold = 1
        root.maxObjectThreshold = 2

        val firstBox = AABB(Vector3f(0f), Vector3f(0.5f))
        root.add(firstBox)

        val secondBox = AABB(Vector3f(-0.5f), Vector3f(1f))
        root.add(secondBox)

        var intersectionCount = 0
        root.depthFirstSearch { a, b ->
            if(a.intersects(b))
                intersectionCount++
        }
        assertEquals(1, intersectionCount)
    }

}