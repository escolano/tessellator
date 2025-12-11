#pragma once

#include "types/Mesh.h"
#include "types/Options.h"
#include "types/Progress.h"
#include "MesherBase.h"

namespace meshlib::meshers {

class StaircaseMesher : public MesherBase {
public:
	StaircaseMesher(const Mesh& in, int decimalPlacesInCollapser = 4);
	virtual ~StaircaseMesher() = default;
	Mesh mesh() const;
private:

	Mesh surfaceMesh_;

	virtual Mesh buildSurfaceMesh(const Mesh& inputMesh, const Mesh& volumeSurface);
	void process(Mesh&) const;
	Options opts_;
};

}
