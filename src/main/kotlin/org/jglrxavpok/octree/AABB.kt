package org.jglrxavpok.octree

import org.joml.Vector3f

class AABB(val min: Vector3f, val max: Vector3f): OctreeData {

    override val boundingBox = this

    fun intersects(other: AABB): Boolean {
        if(min.x() >= other.max.x())
            return false
        if(min.y() >= max.y())
            return false
        if(max.x() <= min.x())
            return false
        if(max.y() <= min.y())
            return false
        return true
    }

    fun fullyContains(other: AABB): Boolean {
        if(!intersects(other))
            return false
        if(other.max.x > max.x)
            return false
        if(other.max.y > max.y)
            return false
        if(other.max.z > max.z)
            return false

        if(other.min.x < min.x)
            return false
        if(other.min.y < min.y)
            return false
        if(other.min.z < min.z)
            return false
        return true
    }

    override fun toString(): String {
        return "AABB[min = $min; max = $max; hashCode = ${hashCode()}]"
    }

    fun copy() = AABB(Vector3f(min), Vector3f(max))
}