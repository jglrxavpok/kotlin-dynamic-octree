package org.jglrxavpok.octree

import org.joml.Vector3f

class Octree<Data: OctreeData>(val region: AABB, val parent: Octree<Data>?) {

    private lateinit var octants: Array<Octree<Data>>

    var maxObjectThreshold = 10
    var minObjectThreshold = 2
    val center = Vector3f(region.max).add(region.min).div(2f) // middle
    val data = hashSetOf<Data>()
    var isLeaf = true
        private set

    private val overlappingPercent = 0.1f

    private fun split() {
        isLeaf = false
        octants = Array(8) { i ->
            when(i) {
                0 -> octantFromAABB(region.min, center)
                1 -> octantFromAABB(Vector3f(center.x(), region.min.y(), region.min.z()), Vector3f(region.max.x(), center.y(), center.z()))
                2 -> octantFromAABB(Vector3f(center.x(), region.min.y(), center.z()), Vector3f(region.max.x(), center.y(), region.max.z()))
                3 -> octantFromAABB(Vector3f(region.min.x(), region.min.y(), center.z()), Vector3f(center.x(), center.y(), region.max.z()))
                4 -> octantFromAABB(Vector3f(region.min.x(), center.y(), region.min.z()), Vector3f(center.x(), region.max.y(), center.z()))
                5 -> octantFromAABB(Vector3f(center.x(), center.y(), region.min.z()), Vector3f(region.max.x(), region.max.y(), center.z()))
                6 -> octantFromAABB(center, region.max)
                7 -> octantFromAABB(Vector3f(region.min.x(), center.y(), center.z()), Vector3f(center.x(), region.max.y(), region.max.z()))
                else -> error("received $i but should be <= 7")
            }
        }
    }

    private fun octantFromAABB(min: Vector3f, max: Vector3f): Octree<Data> {
        val dirToCenterFromMin by lazy { Vector3f() }
        dirToCenterFromMin.set(center).sub(min).mul(overlappingPercent)


        val dirToCenterFromMax by lazy { Vector3f() }
        dirToCenterFromMax.set(center).sub(max).mul(overlappingPercent)

        val newMin by lazy { Vector3f() }
        val newMax by lazy { Vector3f() }
        newMin.set(min).add(dirToCenterFromMax) // from max on purpose, creates overlap between adjacent cubes
        newMax.set(max).add(dirToCenterFromMin)
        val result = Octree(AABB(Vector3f(newMin), Vector3f(newMax)), this)
        result.minObjectThreshold = minObjectThreshold
        result.maxObjectThreshold = maxObjectThreshold
        return result
    }

    fun add(element: Data): Boolean {
        val box = element.boundingBox
        if( ! region.fullyContains(box)) {
            return false
        }
        return when {
            isLeaf && countObjects() < maxObjectThreshold-1 -> {
                data.add(element)
                true
            }
            isLeaf && countObjects() >= maxObjectThreshold-1 -> {
                split()
                // send old elements to its children
                for(oldElement in data) {
                    data.remove(oldElement)
                    add(oldElement)
                }
                add(element)
            }
            else -> { // not a leaf
                val foundSlotInChildren = octants.any { it.add(element) }
                if(!foundSlotInChildren)
                    data.add(element)
                true
            }
        }
    }

    fun moveElement(element: Data): Boolean {
        val couldFindPlace = moveElementInTree(element)
        if(!couldFindPlace) {
            return parent?.add(element) ?: false
        }
        return true
    }

    fun deleteElement(element: Data) {
        when {
            isLeaf -> data.remove(element)
            !isLeaf && countObjects() > minObjectThreshold -> {
                if(element in data)
                    data.remove(element)
                for(child in octants)
                    child.deleteElement(element)
            }
            else -> { // not a leaf but has less elements than required threshold
                if(mergeChildren()) {
                    parent?.deleteElement(element)
                } else {
                    deleteElement(element)
                }
            }
        }
    }

    /**
     * Returns true if parent has to merge its children too
     */
    private fun mergeChildren(): Boolean {
        isLeaf = true
        for(child in octants) {
            data.addAll(child.data)
            child.data.clear()
        }
        if(parent != null && parent.countObjects() < minObjectThreshold) {
            return true
        }
        return false
    }

    /**
     * Returns true if the element found a place
     */
    private fun moveElementInTree(element: Data): Boolean {
        if(element in data)
            deleteElement(element)
        val box = element.boundingBox
        if(region.fullyContains(box)) {
            if(!isLeaf) { // check children first
                for(subtree in octants) {
                    val couldMove = subtree.moveElementInTree(element)
                    if(couldMove)
                        return true
                }
            }
            add(element)
            return true
        }
        return false
    }

    fun countObjects(): Int {
        if(isLeaf)
            return data.size
        return data.size + octants.sumBy { it.countObjects() }
    }

    operator fun contains(element: Data): Boolean {
        if(isLeaf)
            return element in data
        return octants.any { element in it }
    }

    fun findWhereItIs(element: Data): Octree<Data> {
        if(isLeaf && element in data)
            return this
        for(child in octants) {
            if(element in child) {
                return child.findWhereItIs(element)
            }
        }
        if(element in data)
            return this
        error("Could not find element in tree")
    }

    fun listElements(dest: MutableList<Data>) {
        dest.addAll(data)
        if(!isLeaf)
            octants.forEach { it.listElements(dest) }
    }

    fun depthFirstSearch(action: (Data, Data) -> Unit) {
        if(isLeaf) {
            for(firstElement in data) {
                for(secondElement in data) {
                    if(firstElement != secondElement)
                        action(firstElement, secondElement)
                }
            }
        } else {
            val inSubTrees by lazy { mutableListOf<Data>() }
            inSubTrees.clear()
            for(child in octants) {
                if(data.size > 0)
                    child.listElements(inSubTrees) // TODO: potential optimisation by merging the two operations
                child.depthFirstSearch(action)
            }
            for(firstElement in data) {
                for(secondElement in inSubTrees) {
                    action(firstElement, secondElement)
                }
            }
        }
    }

}