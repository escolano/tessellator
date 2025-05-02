#include "StructuredMesher.h"

#include <iostream>


#include "core/Slicer.h"
#include "core/Collapser.h"
#include "core/Staircaser.h"

#include "utils/RedundancyCleaner.h"
#include "utils/MeshTools.h"
#include "utils/GridTools.h"

namespace meshlib::meshers {

using namespace utils;
using namespace core;
using namespace meshTools;

StructuredMesher::StructuredMesher(const Mesh& inputMesh,const Options& options) :
    MesherBase(inputMesh),opts_(options)
{
    opts_.progress.setSections({0.05,0.95});//weights must sum 1.0 and must match the amount of newSection/endSection of this algorithm
    log("Preparing surfaces.");
    opts_.progress.newSection("Preparing surfaces.",0);
    surfaceMesh_ = buildMeshFilteringElements(inputMesh, isNotTetrahedron);
    opts_.progress.endSection();
    log("Processing surface mesh.");
    opts_.progress.newSection("Processing surface mesh.",6);
    process(surfaceMesh_);
    opts_.progress.endSection();
    log("Surface mesh built succesfully.", 1);
}

Mesh StructuredMesher::buildSurfaceMesh(const Mesh& inputMesh, const Mesh & volumeSurface)
{
    auto resultMesh = buildMeshFilteringElements(inputMesh, isNotTetrahedron);
    mergeMesh(resultMesh, volumeSurface);
    return resultMesh;
}

void StructuredMesher::process(Mesh& mesh) const
{
    
    const auto slicingGrid{ buildSlicingGrid(originalGrid_, enlargedGrid_) };
    
    if (mesh.countElems() == 0) {
        mesh.grid = slicingGrid;
        return;
    }

    log("Slicing.", 1);
    opts_.progress.newTask("Slicing.");
    mesh.grid=slicingGrid;
    opts_.progress.endTask();
    
    mesh = Slicer{ mesh }.getMesh();
    
    logNumberOfTriangles(countMeshElementsIf(mesh, isTriangle));

    log("Collapsing.", 1);
    opts_.progress.newTask("Collapsing.");
    mesh = Collapser(mesh,opts_.decimalPlacesInCollapser).getMesh();
    opts_.progress.endTask();

    logNumberOfTriangles(countMeshElementsIf(mesh, isTriangle));
    
    log("Staircasing.", 1);
    opts_.progress.newTask("Staircasing.");
    mesh = Staircaser(mesh).getMesh();
    opts_.progress.endTask();

    logNumberOfQuads(countMeshElementsIf(mesh, isQuad));
    logNumberOfLines(countMeshElementsIf(mesh, isLine));

    log("Removing repeated and overlapping elements.", 1);
    opts_.progress.newTask("Removing repeated and overlapping elements.");
    RedundancyCleaner::removeOverlappedDimensionOneAndLowerElementsAndEquivalentSurfaces(mesh);
    opts_.progress.endTask();

    logNumberOfQuads(countMeshElementsIf(mesh, isQuad));
    logNumberOfLines(countMeshElementsIf(mesh, isLine));
    
    log("Recovering original grid size.", 1);
    opts_.progress.newTask("Recovering original grid size.");
    reduceGrid(mesh, originalGrid_);
    opts_.progress.endTask();

    log("Converting relative to absolute coordinates.", 1);
    opts_.progress.newTask("Converting relative to absolute coordinates.",1);
    utils::meshTools::convertToAbsoluteCoordinates(mesh);
    opts_.progress.endTask();
    
    logNumberOfQuads(countMeshElementsIf(mesh, isQuad));
    logNumberOfLines(countMeshElementsIf(mesh, isLine));

}


Mesh StructuredMesher::mesh() const
{
    return surfaceMesh_;
}

}