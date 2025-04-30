#pragma once

#include "types/Mesh.h"
#include "types/Options.h"
#include "types/Progress.h"
#include "MesherBase.h"

namespace meshlib::meshers {

class StructuredMesher : public MesherBase {
public:
	StructuredMesher(const Mesh& in,const Options& options);
	virtual ~StructuredMesher() = default;
	Mesh mesh() const;
private:

	Mesh surfaceMesh_;

	virtual Mesh buildSurfaceMesh(const Mesh& inputMesh, const Mesh& volumeSurface);
	void process(Mesh&) const;
	Options opts_;
};

}
