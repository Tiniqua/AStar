#pragma once

#define no_init_all // Removes VS 2017 error message. Bug fixed in VS 2019.

#include "Common/d3dApp.h"
#include "Common/MathHelper.h"
#include "Common/UploadBuffer.h"
#include "Common/GeometryGenerator.h"
#include "Common/Camera.h"
#include "FrameResource.h"
#include "AudioSImple.h"
#include "RenderItem.h"
#include "CombatController.h"
#include "SpriteManager.h"
#include "TextConsole.h"
#include "Player.h"
#include "State.h"
#include "Input.h"
#include "DeltaTimer.h"




class GameApp : public D3DApp, public Singleton<GameApp>
{
public:
	GameApp(HINSTANCE hInstance);
	GameApp(const GameApp& rhs) = delete;
	GameApp& operator=(const GameApp& rhs) = delete;
	~GameApp();

	virtual bool Initialize()override;

	static bool DEBUG;

	int timePhase = 0;
	int setPhase(int phase) { timePhase = phase; }

	// Access to instance data from states and gameobjects
	InstanceData* AddRenderItemInstance(const std::string & renderItemName);
	// When gameobject goes out of scope or removed
	void RemoveRenderItemInstance(const std::string & renderItemName, InstanceData* id);

	GameApp& GetGameApp() { return *this; }
	// For access to lights, camera projection, fog
	PassConstants* GetMainPassCB();

	const POINT& GetLastMousePos()
	{
		return mLastMousePos;
	}
	// Set a camera from a state.
	void SetActiveCamera(Camera* cam)
	{
		assert(cam);
		mpActiveCamera = cam;
	}

	virtual void OnResize()override;

	CD3DX12_GPU_DESCRIPTOR_HANDLE GetSpriteGpuDescHandle(const std::string& textureName);

	void DrawSprite(const Sprite& sprite)
	{
		mSpriteManager.DrawSprite(sprite);
	}

	void DrawString(size_t i, const std::string& output, const XMFLOAT2& pos, bool centre = false);

	void DrawString(const Text& t);

	void ChangeState(const std::string& name);

	State* GetState(const std::string& name);

	const Texture* const GetTexture(const std::string& name)
	{
		if (mTextures.count(name) == 1)
		{
			return mTextures[name].get();
		}
		else
		{
			assert(false);
			return nullptr;
		}
	}

	XMFLOAT2 GetClientSize();
	XMINT2 GetClientSizeU2();

	const UINT GetCbvSrvDescriptorSize() const
	{
		return mCbvSrvDescriptorSize;
	}

	GameAudio& GetAudio()
	{
		return mGameAudio;
	}

	//audio trigger for menu button press
	void PlayClickDownAudio();
	// audio trigger for menu button release. Bool passed if the button is unable take actions
	void PlayClickUpAudio(bool success);

	// Output to viewport
	std::ostringstream mDebugLog;
	UINT mInstanceCount = 0;
	UINT GetMaterialIndex(const std::string& materialName);
	
	const std::string& GetStoryText();
	size_t mStoryIndex = 0;


	// TODO: (REMEMBER) TUT STUFF HERE
	const std::string& GetTutorialText();
	std::string mTutorialText = GC::TUTORIAL_INTRO;
	int menusShown = 0;


private:
	virtual void Update(const GameTimer& gt)override;
	virtual void Draw(const GameTimer& gt)override;

	virtual void OnMouseDown(WPARAM btnState, int x, int y)override;
	virtual void OnMouseUp(WPARAM btnState, int x, int y)override;
	virtual void OnMouseMove(WPARAM btnState, int x, int y)override;

	void OnKeyboardInput(const GameTimer& gt);
	void AnimateMaterials(const GameTimer& gt);
	void UpdateInstanceData(const GameTimer& gt);
	void UpdateMaterialBuffer(const GameTimer& gt);
	void UpdateMainPassCB(const GameTimer& gt);

	void LoadTexture(const std::string& name, const std::wstring& filename);
	void LoadTextures();
	void BuildRootSignature();
	void BuildDescriptorHeaps();
	void BuildShadersAndInputLayout();
	void BuildBoxGeometry();
	void BuildObjGeometry(const std::string& filepath, const std::string& geoName,const std::string& subMeshName);
	void BuildTileGeometry();
	void BuildSwordGeometry();
	void BuildPlayerGeometry();
	void BuildBuildingGeometry();
	void BuildPSOs();
	void BuildFrameResources();
	void BuildMaterial(int& index, int texIndex, const std::string& name, float roughness = 0.5f, const DirectX::XMFLOAT4& diffuseAlbedo = { 1.0f, 1.0f, 1.0f, 1.0f }, const DirectX::XMFLOAT3& fresnel = { 0.05f, 0.05f, 0.05f });
	void BuildMaterials();
	//Auto increments obj cb index
	std::unique_ptr<RenderItem> BuildRenderItem(UINT& objCBindex,const std::string& geoName, const std::string& subGeoName);
	void BuildRenderItems();
	void DrawRenderItems(ID3D12GraphicsCommandList* cmdList, const std::vector<RenderItem*>& ritems);

public:
	std::array<const CD3DX12_STATIC_SAMPLER_DESC, 6> GetStaticSamplers();
private:

	std::vector<std::unique_ptr<FrameResource>> mFrameResources;
	FrameResource* mCurrFrameResource = nullptr;
	int mCurrFrameResourceIndex = 0;

	UINT mCbvSrvDescriptorSize = 0;

	Microsoft::WRL::ComPtr<ID3D12RootSignature> mRootSignature = nullptr;

	Microsoft::WRL::ComPtr<ID3D12DescriptorHeap> mSrvDescriptorHeap = nullptr;

	std::unordered_map<std::string, std::unique_ptr<MeshGeometry>> mGeometries;
	std::unordered_map<std::string, std::unique_ptr<Material>> mMaterials;
	std::unordered_map<std::string, std::unique_ptr<Texture>> mTextures;
	std::unordered_map<std::string, Microsoft::WRL::ComPtr<ID3DBlob>> mShaders;
	std::unordered_map<std::string, Microsoft::WRL::ComPtr<ID3D12PipelineState>> mPSOs;

	std::vector<D3D12_INPUT_ELEMENT_DESC> mInputLayout;

	// List of all the render items.
	std::unordered_map<std::string, std::unique_ptr<RenderItem>> mAllRitems;	//Makes it easier to know what you're accessing (i.e "Tiles" rather than 0)

	// Render items divided by PSO.
	std::vector<RenderItem*> mOpaqueRitems;


	bool mFrustumCullingEnabled = true;

	DirectX::BoundingFrustum mCamFrustum;

	PassConstants mMainPassCB;

	//Camera mCamera;
	Camera* mpActiveCamera = nullptr;

	POINT mLastMousePos;

	//Audio
	GameAudio mGameAudio;
	DeltaTimer mAudioTimer; //Changing track after a while

	//States
	StateManager mStateManager;

	//Viewport Sprites
	SpriteManager mSpriteManager;



};