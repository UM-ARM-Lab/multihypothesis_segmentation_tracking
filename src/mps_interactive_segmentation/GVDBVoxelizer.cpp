//
// Created by pricear on 2020-04-24.
//

#include "mps_interactive_segmentation/GVDBVoxelizer.h"

#include "gvdb.h"
#include <GL/glew.h>
#include <fstream>

#include <GL/glx.h>
#include <X11/Xlib.h>

namespace mps
{

nvdb::Model* modelFactory(const shapes::Mesh* data)
{
	const unsigned int numTriangleComponents = 3;
	const unsigned int numPositionComponents = 3;
	const unsigned int numNormalComponents = 3;
	const unsigned int numComponentsPerVertex = numPositionComponents + numNormalComponents;
	auto* model = new nvdb::Model();
	model->modelType = 0; // Polygonal

	model->elemDataType = GL_TRIANGLES;           // What type of GL-renderable primitive does this model contain (e.g., GL_TRIANGLES)
	model->elemCount = (int)data->triangle_count;          // How many of the above primitives are there in the model?
	model->elemArrayOffset = 0;                      // In the element index array, what's the byte offset to the index of the first element to use?
	model->elemStride = numTriangleComponents * sizeof(unsigned int);

	model->vertCount = (int)data->vertex_count;
	model->vertDataType = GL_FLOAT;               // What is the GL data type of each component in the vertex positions?
	model->vertComponents = numPositionComponents;                      // How many components are in each vertex position attribute?
	model->vertOffset = 0;              // What's the byte offset to the start of the first vertex position in the vertex buffer?
	model->vertStride = numComponentsPerVertex
	                    * sizeof(float);           // How many bytes are subsequent verts separated by in the interleaved vertex array created below?

	model->normDataType = GL_FLOAT;               // What is the GL data type of each component in the vertex normals?
	model->normComponents = numNormalComponents;                      // How many components are in each vertex normal attribute?
	model->normOffset = numPositionComponents
	                    * sizeof(float);              // What's the byte offset to the start of the first vertex normal in the vertex buffer?

	model->elemBuffer = static_cast<unsigned int*>(malloc(
		data->triangle_count * numTriangleComponents * sizeof(unsigned int)));
	for (decltype(data->triangle_count) t = 0; t < data->triangle_count; ++t)
	{
		for (unsigned i = 0; i < numTriangleComponents; ++i)
		{
			model->elemBuffer[t * numTriangleComponents + i] = data->triangles[t * numTriangleComponents + i];
		}
	}

	// NB: Vertex position and normal data are interleaved
	model->vertBuffer = static_cast<float*>(malloc(data->vertex_count * numComponentsPerVertex * sizeof(unsigned int)));
	for (decltype(data->vertex_count) v = 0; v < data->vertex_count; ++v)
	{
		for (unsigned i = 0; i < numPositionComponents; ++i)
		{
			model->vertBuffer[v * numComponentsPerVertex + i]
				= (float)data->vertices[v * numPositionComponents + i];
		}

		for (unsigned i = 0; i < numNormalComponents; ++i)
		{
			model->vertBuffer[v * numComponentsPerVertex + numPositionComponents + i]
				= (float)data->vertex_normals[v * numNormalComponents + i];
		}
	}

	return model;
}

struct GLContext
{
	Display* disp;
	Window win;
	GLXFBConfig *fbc;
	XVisualInfo *vi;
	GLXContext ctx;
};

GVDBVoxelizer::GVDBVoxelizer(const mps::VoxelRegion& region, const std::vector<const shapes::Mesh*>& meshes)
 : gvdb(std::make_unique<nvdb::VolumeGVDB>()),
   context(std::make_unique<GLContext>())
{
//	glewExperimental = GL_TRUE;
//	glewInit();

	typedef GLXContext (*glXCreateContextAttribsARBProc)
		(Display*, GLXFBConfig, GLXContext, Bool, const int*);
	context->disp = XOpenDisplay(nullptr);
	context->win = XCreateSimpleWindow(context->disp, DefaultRootWindow(context->disp),
	                          10, 10,   /* x, y */
	                          800, 600, /* width, height */
	                          0, 0,     /* border_width, border */
	                          0);

	static int visual_attribs[] = {
		GLX_RENDER_TYPE, GLX_RGBA_BIT,
		GLX_DRAWABLE_TYPE, GLX_WINDOW_BIT,
		GLX_DOUBLEBUFFER, true,
		GLX_RED_SIZE, 1,
		GLX_GREEN_SIZE, 1,
		GLX_BLUE_SIZE, 1,
		None
	};

	int num_fbc = 0;
	context->fbc = glXChooseFBConfig(context->disp,
	                                     DefaultScreen(context->disp),
	                                     visual_attribs, &num_fbc);
	if (!context->fbc) {
		printf("glXChooseFBConfig() failed\n");
		exit(1);
	}

	/* Create old OpenGL context to get correct function pointer for
	   glXCreateContextAttribsARB() */
	context->vi = glXGetVisualFromFBConfig(context->disp, context->fbc[0]);
	GLXContext ctx_old = glXCreateContext(context->disp, context->vi, 0, GL_TRUE);
	glXCreateContextAttribsARBProc glXCreateContextAttribsARB = 0;
	glXCreateContextAttribsARB =
		(glXCreateContextAttribsARBProc)
			glXGetProcAddress((const GLubyte*)"glXCreateContextAttribsARB");
	/* Destroy old context */
	glXMakeCurrent(context->disp, 0, 0);
	glXDestroyContext(context->disp, ctx_old);
	if (!glXCreateContextAttribsARB) {
		printf("glXCreateContextAttribsARB() not found\n");
		exit(1);
	}

	/* Set desired minimum OpenGL version */
	static int context_attribs[] = {
		GLX_CONTEXT_MAJOR_VERSION_ARB, 4,
		GLX_CONTEXT_MINOR_VERSION_ARB, 4,
		None
	};
	/* Create modern OpenGL context */
	context->ctx = glXCreateContextAttribsARB(context->disp, context->fbc[0], NULL, true,
	                                            context_attribs);
	if (!context->ctx) {
		printf("Failed to create OpenGL context. Exiting.\n");
		exit(1);
	}

	/* Show_the_window
	   --------------- */
	XMapWindow(context->disp, context->win);
	glXMakeCurrent(context->disp, context->win, context->ctx);

	int major = 0, minor = 0;
	glGetIntegerv(GL_MAJOR_VERSION, &major);
	glGetIntegerv(GL_MINOR_VERSION, &minor);
	printf("OpenGL context created.\nVersion %d.%d\nVendor %s\nRenderer %s\n",
	       major, minor,
	       glGetString(GL_VENDOR),
	       glGetString(GL_RENDERER));

	// Initialize GVDB
	gvdb->SetDebug(false);
	gvdb->SetVerbose(false);
	gvdb->SetCudaDevice(GVDB_DEV_FIRST);
	gvdb->Initialize();

	gvdb->AddPath("/home/pricear/local/srcs/gvdb-voxels/source/gvdb_library/shaders/");
	gvdb->AddPath("/home/pricear/local/srcs/gvdb-voxels/source/shared_assets/");

	char scnpath[1024];
	if ( !gvdb->FindFile ( "simple.vert.glsl", scnpath ) )
	{

	}
	gvdb->AddRenderBuf(0, 100, 100, 4);
	gvdb->ValidateOpenGL();
//	gvdb->StartRasterGL();

//	gvdb->SetTransform(Vector3DF(0, 0, 0), Vector3DF(region.resolution, region.resolution, region.resolution), Vector3DF(0, 0, 0), Vector3DF(0, 0, 0));

	// Load mesh data
	gvdb->getScene()->AddModel ( "lucy.obj", 1.0, 0, 0, 0 );
	gvdb->CommitGeometry( 0 );
//	for (size_t i = 0; i < meshes.size(); ++i)
//	{
//		// Add model
//		nvdb::Model* m = modelFactory(meshes[i]);
//		gvdb->getScene()->mModels.push_back(m);
//
//		// Push model to gpu
//		gvdb->CommitGeometry(i);
//	}

	// Configure Voxelization
	gvdb->Configure(3, 3, 3, 3, 5);
	gvdb->SetChannelDefault(16, 16, 1);
	gvdb->AddChannel(0, T_FLOAT, 1); // Add voxel channel


	// Needed?
	gvdb->getScene()->SetSteps ( 0.25f, 16.f, 0.25f );			// Set raycasting steps
	gvdb->getScene()->SetVolumeRange ( 0.25f, 0.0f, 1.0f );		// Set volume value range
	gvdb->getScene()->SetExtinct ( -1.0f, 1.1f, 0.f );			// Set volume extinction	
	gvdb->getScene()->SetCutoff ( 0.005f, 0.005f, 0.f );
	gvdb->getScene()->SetShadowParams ( 0, 0, 0 );
	gvdb->getScene()->LinearTransferFunc ( 0.0f, 0.5f, Vector4DF(0,0,0,0), Vector4DF(1.f,1.f,1.f,0.5f) );
	gvdb->getScene()->LinearTransferFunc ( 0.5f, 1.0f, Vector4DF(1.f,1.f,1.f,0.5f), Vector4DF(1,1,1,0.8f) );
	gvdb->CommitTransferFunc ();
	gvdb->getScene()->SetBackgroundClr ( 0.1f, 0.2f, 0.4f, 1.0f );
}

std::vector<Vector3DI> getSurfaceSparse(/*const */nvdb::VolumeGVDB& gvdb)
{
	std::vector<Vector3DI> surfVoxels;

	const int nActiveBricks = gvdb.mPool->getPoolUsedCnt(0, 0);

	gvdb.RetrieveVDB();
	auto* vdbInfo = reinterpret_cast<VDBInfo*>(gvdb.getVDBInfo());

	for (int chan = 0 ; chan < gvdb.mPool->getNumAtlas(); chan++ )
	{
		DataPtr atlasData = gvdb.mPool->getAtlas(chan);
		const int chan_type = atlasData.type ;
		const int chan_stride = gvdb.mPool->getSize ( chan_type );  // Could make static in future version
		const int brick_res = vdbInfo->brick_res; // Actual size + 2*apron
		const int apron = atlasData.apron;

		auto* brickMem = reinterpret_cast<float*>(malloc(chan_stride * brick_res * brick_res * brick_res));

		const Vector3DI& atlasBrickDims = atlasData.subdim;

		for (int n=0; n < nActiveBricks; n++ )
		{
			Node* node = gvdb.getNode(0, 0, n);
			if (!node->mFlags) continue;
			std::cerr << node->mPos.x << std::endl;

			int zB = n / (atlasBrickDims.x * atlasBrickDims.y);
			int yB = (n - (zB * atlasBrickDims.x * atlasBrickDims.y))/(atlasBrickDims.x);
			int xB = n - (zB * atlasBrickDims.x * atlasBrickDims.y) - (yB * atlasBrickDims.x);

			// Copy a single brick
			CUDA_MEMCPY3D cp;
			cp.srcMemoryType = CU_MEMORYTYPE_ARRAY;
			cp.srcArray = atlasData.garray;
			cp.srcXInBytes = xB * brick_res * chan_stride;
			cp.srcY = yB * brick_res;
			cp.srcZ = zB * brick_res;
			cp.dstMemoryType = CU_MEMORYTYPE_HOST;
			cp.dstHost = brickMem;
			cp.WidthInBytes = brick_res * chan_stride;
			cp.Height = brick_res;
			cp.Depth = brick_res;
			cudaCheck ( cuMemcpy3D ( &cp ), "Revoxelize", "TestMemcpy", "cuMemcpy3D", "", true);
//			for (int z = apron; z < brick_res - apron; ++z )
//			{
//				for (int y = apron; y < brick_res - apron; ++y)
//				{
//					for (int x = apron; x < brick_res - apron; ++x)
			for (int z = 0; z < brick_res; ++z )
			{
				for (int y = 0; y < brick_res; ++y)
				{
					for (int x = 0; x < brick_res; ++x)
					{
						int i = (z * brick_res * brick_res) + (y * brick_res) + x;
						if (brickMem[i] == 1.0f)
						{
							surfVoxels.push_back(node->mPos + Vector3DI(x, y, z));
						}
					}
				}
			}
		}
		free(brickMem);
	}

	return surfVoxels;
}

void writeFile(const std::vector<Vector3DI>& surfVoxels)
{
	std::ofstream yamlFile("/home/pricear/surf.yaml");
	yamlFile << "header:\n"
	            "  seq: 0\n"
	            "  stamp: now\n"
	            "  frame_id: 'world'\n"
	            "ns: ''\n"
	            "id: 0\n"
	            "type: 6\n"
	            "action: 0\n"
	            "pose:\n"
	            "  position: {x: 0.0, y: 0.0, z: 0.0}\n"
	            "  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}\n"
	            "scale: {x: 1, y: 1, z: 1}\n"
	            "color: {r: 0.0, g: 0.0, b: 0.0, a: 1.0}\n"
	            "lifetime: {secs: 0, nsecs: 0}\n"
	            "frame_locked: true\n"
	            "points:\n";
	for (const auto& v : surfVoxels)
	{
		yamlFile << "- {x: " << v.x/100.0 << ", y: " << v.y/100.0 << ", z: " << v.z/100.0 << "}\n";
	}
//		            "- {x: 0.0, y: 0.0, z: 0.0}\n"
//		            "colors:\n"
//		            "- {r: 0.0, g: 0.0, b: 0.0, a: 0.0}\n"
	yamlFile << "text: ''\n"
	            "mesh_resource: ''\n"
	            "mesh_use_embedded_materials: false\n";
	yamlFile.close();
}

mps::VoxelRegion::VertexLabels GVDBVoxelizer::voxelize(
	const mps::VoxelRegion& region,
	const std::vector<Eigen::Isometry3d>& poses)
{
	mps::VoxelRegion::VertexLabels labels(region.num_vertices(), mps::VoxelRegion::FREE_SPACE);

	float m_part_size = 100.0;
	float m_voxel_size = 0.5;
	Vector3DF	m_pivot;
	m_pivot.Set(0.3f, 0.45f, 0.3f);

	Matrix4F xform, scale, mat;
	xform.Identity();
//	scale.Scale(1.0/region.resolution, 1.0/region.resolution, 1.0/region.resolution);

	// Complete poly-to-voxel transform:
	//    X = S(partsize) S(1/voxelsize) Torigin R
	//  (remember, matrices are multiplied left-to-right but applied conceptually right-to-left)
	mat.Scale(m_part_size, m_part_size, m_part_size);
	xform *= mat;									// 4th) Apply part size
	mat.Scale(1 / m_voxel_size, 1 / m_voxel_size, 1 / m_voxel_size);
	xform *= mat;									// 3rd) Apply voxel size (scale by inverse of this)
	mat.Translate(m_pivot.x, m_pivot.y, m_pivot.z);	// 2nd) Move part so origin is at bottom corner
	xform *= mat;
	mat.RotateZYX(Vector3DF(0, -10, 0));			// 1st) Rotate part about the geometric center
	xform *= mat;

	// Set transform for rendering
	// Scale the GVDB grid by voxelsize to render the model in our desired world coordinates
	gvdb->SetTransform(Vector3DF(0, 0, 0), Vector3DF(m_voxel_size, m_voxel_size, m_voxel_size), Vector3DF(0, 0, 0), Vector3DF(0, 0, 0));


	for (int i = 0; i < gvdb->getScene()->getNumModels(); ++i)
	{
		Eigen::Matrix<float, 4, 4, Eigen::RowMajor> tempPose(poses[i].matrix().cast<float>());

//		xform.Identity();
//		xform *= scale;
//		xform *= Matrix4F(tempPose.data());

		// Add model
		nvdb::Model* m = gvdb->getScene()->getModel(i);

		gvdb->SolidVoxelize(0, m, &xform, 1.0, 0.5);

		const int nActiveBricks = gvdb->mPool->getPoolUsedCnt(0, 0);
		if (nActiveBricks == 0)
		{
			std::cerr << "Warning: No bricks generated for model " << i << std::endl;
			continue;
		}

		auto dat = getSurfaceSparse(*gvdb);

		writeFile(dat);

		for (const auto& d : dat)
		{
			Eigen::Vector3d p = Eigen::Vector3d(d.x, d.y, d.z) * region.resolution;
			auto q = coordToVertexDesc(region.resolution, region.regionMin, p);
			if (region.isInRegion(p))
			{
				labels[region.index_of(q)] = i+1;
			}
		}

	}

	return labels;
}

GVDBVoxelizer::~GVDBVoxelizer() = default;

}