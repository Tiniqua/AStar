//***************************************************************************************
// InstancingAndCullingApp.cpp by Frank Luna (C) 2015 All Rights Reserved.
//***************************************************************************************

#include <xaudio2.h>
#include <functional>
#include "GameApp.h"
#include "SimpleMath.h"
#include "OBJ_Loader.h"
#include <cassert>
#include <sstream>

using Microsoft::WRL::ComPtr;
using namespace DirectX;
using namespace DirectX::PackedVector;

#pragma comment(lib, "d3dcompiler.lib")
#pragma comment(lib, "D3D12.lib")

const int gNumFrameResources = 3; //todo move to GC namespace in its own header

bool GameApp::DEBUG = false;

// todo remove
std::string PrintMatrix(XMMATRIX& xmm)
{
	XMFLOAT4X4 m;
	DirectX::XMStoreFloat4x4(&m, xmm);

	std::ostringstream o;

	o
		<< m._11 << "      "
		<< m._12 << "      "
		<< m._13 << "      "
		<< m._14 << "\n"
		<< m._21 << "      "
		<< m._22 << "      "
		<< m._23 << "      "
		<< m._24 << "\n"
		<< m._31 << "      "
		<< m._32 << "      "
		<< m._33 << "      "
		<< m._34 << "\n"
		<< m._41 << "      "
		<< m._42 << "      "
		<< m._43 << "      "
		<< m._44 << "\n"
		;
	return o.str();
}

GameApp::GameApp(HINSTANCE hInstance)
	: D3DApp(hInstance)
{
}

GameApp::~GameApp()
{
	OutputDebugStringA("~GameApp before flush \n");

	if (md3dDevice != nullptr)
		FlushCommandQueue();

	OutputDebugStringA("~GameApp after flush \n");
}

bool GameApp::Initialize()
{
	if (!D3DApp::Initialize())
		return false;



	// Reset the command list to prep for initialization commands.
	ThrowIfFailed(mCommandList->Reset(mDirectCmdListAlloc.Get(), nullptr));

	// Get the increment size of a descriptor in this heap type.  This is hardware specific, 
	// so we have to query this information.
	mCbvSrvDescriptorSize = md3dDevice->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV);



	//Audio setup
	{
		mGameAudio.Init();

		// Creates an 'engine' for sound effects. 
		mGameAudio.CreateEngine("sfx", AUDIO_ENGINE_TYPE::SFX);
		mGameAudio.LoadSound("sfx", "chord", L"Data/Sounds/chord.wav");
		mGameAudio.LoadSound("sfx", "playerFootstep", L"Data/Sounds/playerFootstep.wav");
		mGameAudio.LoadSound("sfx", "EnemyDie1", L"Data/Sounds/enemyrDie01.wav");
		mGameAudio.LoadSound("sfx", "EnemyAttack1", L"Data/Sounds/enemyrAttack01.wav");
		mGameAudio.LoadSound("sfx", "EnemyHit1", L"Data/Sounds/enemyrHit01.wav");
		mGameAudio.LoadSound("sfx", "playerFootstepStone", L"Data/Sounds/stone2.wav");
		mGameAudio.LoadSound("sfx", "playerFootstepGrass", L"Data/Sounds/grass2.wav");
		mGameAudio.LoadSound("sfx", "playerDie", L"Data/Sounds/death sound2.wav");
		mGameAudio.LoadSound("sfx", "playerAttack", L"Data/Sounds/swingWoosh.wav");

		// Plays up to 20 instances at once. SFX only
		mGameAudio.SetCacheSize("sfx", 20);
		// New instance to play when cache is full. Oldest instance removed SFX only
		mGameAudio.ForceAudio("sfx", true);

		mGameAudio.CreateEngine("trader", AUDIO_ENGINE_TYPE::SFX);
		mGameAudio.LoadSound("trader", "trader01", L"Data/Sounds/trader01.wav");
		mGameAudio.LoadSound("trader", "trader02", L"Data/Sounds/trader02.wav");
		mGameAudio.LoadSound("trader", "trader03", L"Data/Sounds/trader03.wav");
		mGameAudio.LoadSound("trader", "trader04", L"Data/Sounds/trader04.wav");
		mGameAudio.LoadSound("trader", "trader05", L"Data/Sounds/trader05.wav");
		mGameAudio.LoadSound("trader", "trader06", L"Data/Sounds/trader06.wav");
		// stops talking before saying something else
		mGameAudio.ForceAudio("trader", true);
		mGameAudio.SetCacheSize("trader", 1);

		mGameAudio.CreateEngine("player", AUDIO_ENGINE_TYPE::SFX);
		mGameAudio.LoadSound("player", "playerHit01", L"Data/Sounds/playerHit01.wav");
		// player can only make one sound at a time
		mGameAudio.ForceAudio("player", true);
		mGameAudio.SetCacheSize("player", 1);

		mGameAudio.CreateEngine("ui", AUDIO_ENGINE_TYPE::SFX);
		// Loads a sound onto an 'engine'
		mGameAudio.LoadSound("ui", "clickDown", L"Data/Sounds/clickDown.wav");
		mGameAudio.LoadSound("ui", "clickUpFail", L"Data/Sounds/clickUpFail.wav");
		mGameAudio.LoadSound("ui", "clickUpSuccess", L"Data/Sounds/clickUpSuccess.wav");
		mGameAudio.SetCacheSize("ui", 30);
		// New instance to play when cache is full. Oldest instance removed SFX only
		mGameAudio.ForceAudio("ui", true);

		//Music 'engine'
		mGameAudio.CreateEngine("music", AUDIO_ENGINE_TYPE::MUSIC);
		// Loads sounds the same way
		mGameAudio.LoadSound("music", "actionMusic", L"Data/Sounds/Main game theme bugaloo.wav");
		mGameAudio.LoadSound("music", "menuMusic", L"Data/Sounds/Spooky_Title_screen_Loop.wav");
		// Time it takes to fade between tracks when Play() is called
		mGameAudio.SetFade("music", 1.0f);
		// Plays audio from 'music' engine. No need to specify engine
		mGameAudio.Play("menuMusic", nullptr, true);

		mGameAudio.CreateEngine("ambient", AUDIO_ENGINE_TYPE::MUSIC);
		mGameAudio.LoadSound("ambient", "ambientCrows", L"Data/Sounds/ambientCrows.wav");
		mGameAudio.LoadSound("ambient", "ambientWind", L"Data/Sounds/ambientWind.wav");
		mGameAudio.SetFade("ambient", 5.0f);

		float volume = 0.2f;
		mGameAudio.SetEngineVolume("music", volume * 0.5f);
		mGameAudio.SetEngineVolume("sfx", volume);
		mGameAudio.SetEngineVolume("trader", volume);
		mGameAudio.SetEngineVolume("ui", volume);
		mGameAudio.SetEngineVolume("player", volume);
		mGameAudio.SetEngineVolume("ambient", volume*0.5f);
	}

	LoadTextures();
	BuildRootSignature();
	BuildDescriptorHeaps();
	BuildShadersAndInputLayout();
	BuildBoxGeometry();
	// For each geo object
	//	add to mAllRitems in BuildRenderItems()
	// Change differentRenderObjectsNum in frame resource to reflect this. 
	BuildObjGeometry("Data/Models/weapon00.obj", "tempSwordGeo", "tempSword");// loads obj
	BuildObjGeometry("Data/Models/charCloak.obj", "tempPlayerGeo", "tempPlayer");//tempPlayer.obj
	BuildObjGeometry("Data/Models/charBlob.obj", "tempEnemyGeo", "tempEnemy");
	BuildObjGeometry("Data/Models/flatTile.obj","floorTileGeo", "floorTile" ); //quad rather than cube
	BuildObjGeometry("Data/Models/building04.obj","traderGeo", "trader" ); //quad rather than cube
	BuildObjGeometry("Data/Models/building03.obj", "buildingGeo", "building"); 
	BuildObjGeometry("Data/Models/trader.obj","traderGeo", "trader" ); //quad rather than cube
	BuildObjGeometry("Data/Models/particle.obj", "particle", "particle");		//Particle used for enemy attack
	BuildObjGeometry("Data/Models/plantPotato.obj","potatoGeo", "potato" ); // plant
	BuildObjGeometry("Data/Models/carBody.obj","carBodyGeo", "carBody" );  // 'trader' main objective. requires 1x1x1 bounding box
	BuildObjGeometry("Data/Models/carWheel.obj","carTireGeo", "carTire" ); // two instances 'trader' main objective  either side of car body. requires 1x1x1 bounding box
	BuildObjGeometry("Data/Models/charGhoul.obj","enemyGhoulGeo", "enemyGhoul" ); // two instances 'trader' main objective  either side of car body. requires 1x1x1 bounding box

	BuildSwordGeometry();
	BuildPlayerGeometry();
	BuildMaterials();
	BuildRenderItems();
	// Sets up all states. Requires render items

	mStateManager.Init(); 
	BuildFrameResources();
	BuildPSOs();

	Input::Get().Initialize();


	// Normally called OnSize() at start but no camera is created until states initialised.
	if (mpActiveCamera)
	{
		mpActiveCamera->SetLens(0.25f*MathHelper::Pi, AspectRatio(), 1.0f, 1000.0f);
		BoundingFrustum::CreateFromMatrix(mCamFrustum, mpActiveCamera->GetProj());
	}

	// Execute the initialization commands.
	ThrowIfFailed(mCommandList->Close());
	ID3D12CommandList* cmdsLists[] = { mCommandList.Get() };
	mCommandQueue->ExecuteCommandLists(_countof(cmdsLists), cmdsLists);

	// Wait until initialization is complete.
	FlushCommandQueue();

	return true;
}

InstanceData* GameApp::AddRenderItemInstance(const std::string & renderItemName)
{
	if (mAllRitems.count(renderItemName) == 1)
	{
		// Had an issue with pushback so using resize. 
		//Todo: need to test large number of instances since using a deque. 
		mAllRitems[renderItemName]->Instances.resize(mAllRitems[renderItemName]->Instances.size() + 1);
		++mAllRitems[renderItemName]->InstanceCount;
		++mInstanceCount;
		return &mAllRitems[renderItemName]->Instances.back();

	}
	else
	{
		OutputDebugStringA("renderItemName doesnt exist");
		assert(false);
		return nullptr;
	}
}

void GameApp::RemoveRenderItemInstance(const std::string & renderItemName, InstanceData* id)
{

	if (mAllRitems.count(renderItemName) == 1)
	{
		if (id)
		{
			std::ostringstream oss;
			oss << "Comparing instance data  \n" << renderItemName << "\n";

			std::list<InstanceData>::iterator it = mAllRitems[renderItemName]->Instances.begin();
			bool found = false;
			while (it != mAllRitems[renderItemName]->Instances.end())
			{
				oss << " &(*it) = " << &(*it) << " InstanceData* id = " << id << "\n";

				if (id == &(*it)    )
				{
					mAllRitems[renderItemName]->Instances.erase(it);

					// update instance count
					--mAllRitems[renderItemName]->InstanceCount;
					--mInstanceCount;

					oss << "match and erased. " << renderItemName
						<< " instance count: " << mAllRitems[renderItemName]->InstanceCount
						<< " overal instance count: "  << mInstanceCount << "\n"
						;
					found = true;
					it = mAllRitems[renderItemName]->Instances.end();
				}
				else
				{
					++it;
				}
			}

			oss << "post loop...\n";

			if (!found)
			{
				oss << "NOT ";
			}
			oss << " FOUND AND REMOVED \n";

			 //removes instance data from list
			/*mAllRitems[renderItemName]->Instances.erase(std::remove_if(
				mAllRitems[renderItemName]->Instances.begin(),
				mAllRitems[renderItemName]->Instances.end(), [&](InstanceData& i)
			{
				OutputDebugStringA("comparing InstanceData  ");
				if (id == &i)
				{
					return id == &i;
				}
				else
				{
					return false;
				}
			}), mAllRitems[renderItemName]->Instances.end());

			 decrement count
*/

			OutputDebugStringA(oss.str().c_str());

		}
	}
	else
	{
		OutputDebugStringA("renderItemName doesnt exist");
		assert(false);
	}
}

void GameApp::OnResize()
{
	D3DApp::OnResize();

	//assert(mpActiveCamera);

	if (mpActiveCamera)
	{ 
		mpActiveCamera->SetLens(0.25f*MathHelper::Pi, AspectRatio(), 1.0f, 1000.0f);

		BoundingFrustum::CreateFromMatrix(mCamFrustum, mpActiveCamera->GetProj());

	}
}

void GameApp::DrawString(size_t i, const std::string & output, const XMFLOAT2 & pos, bool centre)
{
	mSpriteManager.DrawString(i, output.c_str(), pos, centre);
}

void GameApp::DrawString(const Text & t)
{
	mSpriteManager.DrawString(t);
}

void GameApp::ChangeState(const std::string & name)
{
	mStateManager.ChangeState(name);
}

State * GameApp::GetState(const std::string & name)
{
	return mStateManager.GetState(name);
}

XMFLOAT2 GameApp::GetClientSize()
{
	return {(float)mClientWidth, (float)mClientHeight};
}

XMINT2 GameApp::GetClientSizeU2()
{
	return {(int) mClientWidth,(int)mClientHeight };
}

void GameApp::PlayClickDownAudio()
{
	mGameAudio.Play("clickDown");
}

void GameApp::PlayClickUpAudio(bool success)
{
	if (success)
	{
		mGameAudio.Play("clickUpSuccess");
	}
	else
	{
		mGameAudio.Play("clickUpFail");
	}
}

UINT GameApp::GetMaterialIndex(const std::string & materialName)
{

	UINT i = 0;
	std::find_if(mMaterials.begin(), mMaterials.end(), [&](auto& p)
	{
		if (p.first == materialName)
		{
			return true;
		}
		else
		{
			++i;
			return false;
		}
	});

	assert(i < mMaterials.size()); // out of bounds. Material isnt there

	return i;
}

void GameApp::Update(const GameTimer& gt)
{
	assert(mpActiveCamera);

	OnKeyboardInput(gt);
	Input::Get().Update();

	// Cycle through the circular frame resource array.
	mCurrFrameResourceIndex = (mCurrFrameResourceIndex + 1) % gNumFrameResources;
	mCurrFrameResource = mFrameResources[mCurrFrameResourceIndex].get();

	// Has the GPU finished processing the commands of the current frame resource?
	// If not, wait until the GPU has completed commands up to this fence point.
	if (mCurrFrameResource->Fence != 0 && mFence->GetCompletedValue() < mCurrFrameResource->Fence)
	{
		HANDLE eventHandle = CreateEventEx(nullptr, false, false, EVENT_ALL_ACCESS);
		ThrowIfFailed(mFence->SetEventOnCompletion(mCurrFrameResource->Fence, eventHandle));
		WaitForSingleObject(eventHandle, INFINITE);
		CloseHandle(eventHandle);
	}


	mDebugLog
		<< "";
		/*<< "\n G) Enemy Instance count: "
		<< mAllRitems[GC::GO_ENEMY]->InstanceCount
		<< " Instance Size(): "
		<< mAllRitems[GC::GO_ENEMY]->Instances.size()
		<< "\n\n"
		;*/

	mStateManager.Update(gt);
	AnimateMaterials(gt);
	UpdateInstanceData(gt);
	UpdateMaterialBuffer(gt);
	UpdateMainPassCB(gt);


	mGameAudio.Update(gt, mpActiveCamera->GetPosition3f(), mpActiveCamera->GetLook3f(), mpActiveCamera->GetUp3f());

}

void GameApp::Draw(const GameTimer& gt)
{
	auto cmdListAlloc = mCurrFrameResource->CmdListAlloc;

	// Reuse the memory associated with command recording.
	// We can only reset when the associated command lists have finished execution on the GPU.
	ThrowIfFailed(cmdListAlloc->Reset());

	// A command list can be reset after it has been added to the command queue via ExecuteCommandList.
	// Reusing the command list reuses memory.
	ThrowIfFailed(mCommandList->Reset(cmdListAlloc.Get(), mPSOs["opaque"].Get()));

	mCommandList->RSSetViewports(1, &mScreenViewport);
	mCommandList->RSSetScissorRects(1, &mScissorRect);
	
	// Indicate a state transition on the resource usage.
	mCommandList->ResourceBarrier(1, &CD3DX12_RESOURCE_BARRIER::Transition(CurrentBackBuffer(),
		D3D12_RESOURCE_STATE_PRESENT, D3D12_RESOURCE_STATE_RENDER_TARGET));

	// Clear the back buffer and depth buffer.
	mCommandList->ClearRenderTargetView(CurrentBackBufferView(), Colors::LightSteelBlue, 0, nullptr);
	mCommandList->ClearDepthStencilView(DepthStencilView(), D3D12_CLEAR_FLAG_DEPTH | D3D12_CLEAR_FLAG_STENCIL, 1.0f, 0, 0, nullptr);

	// Specify the buffers we are going to render to.
	mCommandList->OMSetRenderTargets(1, &CurrentBackBufferView(), true, &DepthStencilView());


	ID3D12DescriptorHeap* descriptorHeaps[] = { mSrvDescriptorHeap.Get() };
	mCommandList->SetDescriptorHeaps(_countof(descriptorHeaps), descriptorHeaps);


	mCommandList->SetGraphicsRootSignature(mRootSignature.Get());

	// Bind all the materials used in this scene.  For structured buffers, we can bypass the heap and 
	// set as a root descriptor.
	auto matBuffer = mCurrFrameResource->MaterialBuffer->Resource();
	mCommandList->SetGraphicsRootShaderResourceView(1, matBuffer->GetGPUVirtualAddress());

	auto passCB = mCurrFrameResource->PassCB->Resource();
	mCommandList->SetGraphicsRootConstantBufferView(2, passCB->GetGPUVirtualAddress());

	// Bind all the textures used in this scene.
	mCommandList->SetGraphicsRootDescriptorTable(3, mSrvDescriptorHeap->GetGPUDescriptorHandleForHeapStart());



	// dont render if its a menu
	if (!mStateManager.IsMenu())
	{
		DrawRenderItems(mCommandList.Get(), mOpaqueRitems);
	}

	mCommandList->SetPipelineState(mPSOs["opaque"].Get());
	// Sprite drawing
	mSpriteManager.DrawBegin(mCommandList.Get(), mScreenViewport);

	// Draws sprites and fonts based on current state
	mStateManager.Draw(gt);

	mSpriteManager.DrawEnd();



	// Indicate a state transition on the resource usage.
	mCommandList->ResourceBarrier(1, &CD3DX12_RESOURCE_BARRIER::Transition(CurrentBackBuffer(),
		D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_PRESENT));

	// Done recording commands.
	ThrowIfFailed(mCommandList->Close());
	// Add the command list to the queue for execution.
	ID3D12CommandList* cmdsLists[] = { mCommandList.Get() };
	mCommandQueue->ExecuteCommandLists(_countof(cmdsLists), cmdsLists);

	// Swap the back and front buffers
	ThrowIfFailed(mSwapChain->Present(0, 0));
	mCurrBackBuffer = (mCurrBackBuffer + 1) % SwapChainBufferCount;

	// Advance the fence value to mark commands up to this fence point.
	mCurrFrameResource->Fence = ++mCurrentFence;

	// Add an instruction to the command queue to set a new fence point. 
	// Because we are on the GPU timeline, the new fence point won't be 
	// set until the GPU finishes processing all the commands prior to this Signal().
	mCommandQueue->Signal(mFence.Get(), mCurrentFence);
}

void GameApp::OnMouseDown(WPARAM btnState, int x, int y)
{
	mLastMousePos.x = x;
	mLastMousePos.y = y;

	mStateManager.OnMouseDown(btnState, x, y);

	SetCapture(mhMainWnd);
}

void GameApp::OnMouseUp(WPARAM btnState, int x, int y)
{
	ReleaseCapture();

	mStateManager.OnMouseUp(btnState, x, y);
}

void GameApp::OnMouseMove(WPARAM btnState, int x, int y)
{
	assert(mpActiveCamera);

	mStateManager.OnMouseMove(btnState, x, y);

	mLastMousePos.x = x;
	mLastMousePos.y = y;
}

void GameApp::OnKeyboardInput(const GameTimer& gt)
{

	mStateManager.OnKeyboardInput(gt);
	
	if (GetAsyncKeyState('1') & 0x8000)
		mFrustumCullingEnabled = true;

	if (GetAsyncKeyState('2') & 0x8000)
		mFrustumCullingEnabled = false;

}

void GameApp::AnimateMaterials(const GameTimer& gt)
{

}

void GameApp::UpdateInstanceData(const GameTimer& gt)
{
	assert(mpActiveCamera);
	XMMATRIX view = mpActiveCamera->GetView();
	XMMATRIX invView = XMMatrixInverse(&XMMatrixDeterminant(view), view);
	
	//mCombatController.Update(mAllRitems);		//Continues rotating the weapon if the player has attacked
	//mPlayer.UpdatePos(mAllRitems);

	int i = 0;					//Makes sure each object with a different geo is using a different instance buffer
	for (auto& e : mAllRitems)
	{
		auto currInstanceBuffer = mCurrFrameResource->InstanceBuffer[i].get();
		const auto& instanceData = e.second->Instances;
		i++;		//Increments the instance buffer for the next geo to be rendered
		int visibleInstanceCount = 0;



		for (auto& instance : instanceData)
		{
			XMMATRIX world = XMLoadFloat4x4(&instance.World);
			XMMATRIX texTransform = XMLoadFloat4x4(&instance.TexTransform);

			XMMATRIX invWorld = XMMatrixInverse(&XMMatrixDeterminant(world), world);

			// View space to the object's local space.
			XMMATRIX viewToLocal = XMMatrixMultiply(invView, invWorld);

			// Transform the camera frustum from view space to the object's local space.
			BoundingFrustum localSpaceFrustum;
			mCamFrustum.Transform(localSpaceFrustum, viewToLocal);

			// Perform the box/frustum intersection test in local space.
			if ((localSpaceFrustum.Contains(e.second->Bounds) != DirectX::DISJOINT) || (mFrustumCullingEnabled == false))
			{
				InstanceData data;
				XMStoreFloat4x4(&data.World, XMMatrixTranspose(world));
				XMStoreFloat4x4(&data.TexTransform, XMMatrixTranspose(texTransform));
				data.MaterialIndex = instance.MaterialIndex;

				// Write the instance data to structured buffer for the visible objects.
				currInstanceBuffer->CopyData(visibleInstanceCount++, data);
			}
		}

		/*

		for (UINT i = 0; i < (UINT)instanceData.size(); ++i)
		{
			XMMATRIX world = XMLoadFloat4x4(&instanceData[i].World);
			XMMATRIX texTransform = XMLoadFloat4x4(&instanceData[i].TexTransform);

			XMMATRIX invWorld = XMMatrixInverse(&XMMatrixDeterminant(world), world);

			// View space to the object's local space.
			XMMATRIX viewToLocal = XMMatrixMultiply(invView, invWorld);

			// Transform the camera frustum from view space to the object's local space.
			BoundingFrustum localSpaceFrustum;
			mCamFrustum.Transform(localSpaceFrustum, viewToLocal);

			// Perform the box/frustum intersection test in local space.
			if ((localSpaceFrustum.Contains(e.second->Bounds) != DirectX::DISJOINT) || (mFrustumCullingEnabled == false))
			{
				InstanceData data;
				XMStoreFloat4x4(&data.World, XMMatrixTranspose(world));
				XMStoreFloat4x4(&data.TexTransform, XMMatrixTranspose(texTransform));
				data.MaterialIndex = instanceData[i].MaterialIndex;

				// Write the instance data to structured buffer for the visible objects.
				currInstanceBuffer->CopyData(visibleInstanceCount++, data);
			}
		}
		*/

		e.second->InstanceCount = visibleInstanceCount;

		std::wostringstream outs;
		outs.precision(2);
		outs << L"Instancing and Culling Demo" <<
			L"    " << e.second->InstanceCount <<
			L" objects visible out of " << e.second->Instances.size();
		mMainWndCaption = outs.str();
	}
}

void GameApp::UpdateMaterialBuffer(const GameTimer& gt)
{
	auto currMaterialBuffer = mCurrFrameResource->MaterialBuffer.get();
	for (auto& e : mMaterials)
	{
		// Only update the cbuffer data if the constants have changed.  If the cbuffer
		// data changes, it needs to be updated for each FrameResource.
		Material* mat = e.second.get();
		if (mat->NumFramesDirty > 0)
		{
			XMMATRIX matTransform = XMLoadFloat4x4(&mat->MatTransform);

			MaterialData matData;
			matData.DiffuseAlbedo = mat->DiffuseAlbedo;
			matData.FresnelR0 = mat->FresnelR0;
			matData.Roughness = mat->Roughness;
			XMStoreFloat4x4(&matData.MatTransform, XMMatrixTranspose(matTransform));
			matData.DiffuseMapIndex = mat->DiffuseSrvHeapIndex;

			currMaterialBuffer->CopyData(mat->MatCBIndex, matData);

			// Next FrameResource need to be updated too.
			mat->NumFramesDirty--;
		}
	}
}

void GameApp::UpdateMainPassCB(const GameTimer& gt)
{
	assert(mpActiveCamera);

	XMMATRIX view = mpActiveCamera->GetView();
	XMMATRIX proj = mpActiveCamera->GetProj();

	XMMATRIX viewProj = XMMatrixMultiply(view, proj);
	XMMATRIX invView = XMMatrixInverse(&XMMatrixDeterminant(view), view);
	XMMATRIX invProj = XMMatrixInverse(&XMMatrixDeterminant(proj), proj);
	XMMATRIX invViewProj = XMMatrixInverse(&XMMatrixDeterminant(viewProj), viewProj);

	XMStoreFloat4x4(&mMainPassCB.View, XMMatrixTranspose(view));
	XMStoreFloat4x4(&mMainPassCB.InvView, XMMatrixTranspose(invView));
	XMStoreFloat4x4(&mMainPassCB.Proj, XMMatrixTranspose(proj));
	XMStoreFloat4x4(&mMainPassCB.InvProj, XMMatrixTranspose(invProj));
	XMStoreFloat4x4(&mMainPassCB.ViewProj, XMMatrixTranspose(viewProj));
	XMStoreFloat4x4(&mMainPassCB.InvViewProj, XMMatrixTranspose(invViewProj));
	mMainPassCB.EyePosW = mpActiveCamera->GetPosition3f();
	mMainPassCB.RenderTargetSize = XMFLOAT2((float)mClientWidth, (float)mClientHeight);
	mMainPassCB.InvRenderTargetSize = XMFLOAT2(1.0f / mClientWidth, 1.0f / mClientHeight);
	mMainPassCB.NearZ = 1.0f;
	mMainPassCB.FarZ = 1000.0f;
	mMainPassCB.TotalTime = gt.TotalTime();
	mMainPassCB.DeltaTime = gt.DeltaTime();

	//mMainPassCB.AmbientLight = { 0.25f, 0.25f, 0.35f, 1.0f };
	//mMainPassCB.AmbientLight = GC::DAWN_COLOUR;

	//mMainPassCB.Lights[0].Direction = { 0.57735f, -0.57735f, 0.57735f };
	mMainPassCB.Lights[0].Direction = { 0.5f, -1.0f, 1.0f }; //normalised in shader
	//mMainPassCB.Lights[0].Strength = GC::DAWN_STRENGTH; // { 1.0f, 0.9f, 0.6f } normal

	auto currPassCB = mCurrFrameResource->PassCB.get();
	currPassCB->CopyData(0, mMainPassCB);
}

// **************************** TEXTURES ****************************************************

void GameApp::LoadTexture(const std::string & name, const std::wstring & filename)
{
	auto texture = std::make_unique<Texture>();
	assert(texture);
	texture->Name = name;
	texture->Filename = filename;
	ThrowIfFailed(DirectX::CreateDDSTextureFromFile12(md3dDevice.Get(),
		mCommandList.Get(), texture->Filename.c_str(),
		texture->Resource, texture->UploadHeap));

	mTextures[texture->Name] = std::move(texture);
}

void GameApp::LoadTextures()
{
	mTextures.reserve(20);
	LoadTexture("bricksTex", L"Data/Textures/textureAtlus.dds"); //1
	//LoadTexture("bricksTex", L"Data/Textures/bricks.dds"); //1
	LoadTexture("stoneTex", L"Data/Textures/stone.dds"); // 2
	LoadTexture("mudTex", L"Data/Textures/LostMeadow_dirt.dds"); // 3
	LoadTexture("iceTex", L"Data/Textures/asphalt.dds"); // 4     ice.dds
	LoadTexture("crateTex", L"Data/Textures/WoodCrate01.dds"); // 5
	LoadTexture("grassTex", L"Data/Textures/grass.dds"); // 6
	LoadTexture("tileTex", L"Data/Textures/tile.dds"); // 7 // POSSIBLE ERROR WITH TEXTURE LOADED INTO POSITION 7
	LoadTexture("defaultTex", L"Data/Textures/white1x1.dds"); // 8
	LoadTexture("uiTex", L"Data/Textures/ui.dds"); // 9
}

void GameApp::BuildRootSignature()
{
	CD3DX12_DESCRIPTOR_RANGE texTable;
	texTable.Init(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, 11, 0, 0); // inc 2 fonts

	// Root parameter can be a table, root descriptor or root constants.
	CD3DX12_ROOT_PARAMETER slotRootParameter[4];

	// Perfomance TIP: Order from most frequent to least frequent.
	slotRootParameter[0].InitAsShaderResourceView(0, 1);
	slotRootParameter[1].InitAsShaderResourceView(1, 1);
	slotRootParameter[2].InitAsConstantBufferView(0);
	slotRootParameter[3].InitAsDescriptorTable(1, &texTable, D3D12_SHADER_VISIBILITY_PIXEL);

	auto staticSamplers = GetStaticSamplers();

	// A root signature is an array of root parameters.
	CD3DX12_ROOT_SIGNATURE_DESC rootSigDesc(4, slotRootParameter,
		(UINT)staticSamplers.size(), staticSamplers.data(),
		D3D12_ROOT_SIGNATURE_FLAG_ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT);

	// create a root signature with a single slot which points to a descriptor range consisting of a single constant buffer
	ComPtr<ID3DBlob> serializedRootSig = nullptr;
	ComPtr<ID3DBlob> errorBlob = nullptr;
	HRESULT hr = D3D12SerializeRootSignature(&rootSigDesc, D3D_ROOT_SIGNATURE_VERSION_1,
		serializedRootSig.GetAddressOf(), errorBlob.GetAddressOf());

	if (errorBlob != nullptr)
	{
		::OutputDebugStringA((char*)errorBlob->GetBufferPointer());
	}
	ThrowIfFailed(hr);

	ThrowIfFailed(md3dDevice->CreateRootSignature(
		0,
		serializedRootSig->GetBufferPointer(),
		serializedRootSig->GetBufferSize(),
		IID_PPV_ARGS(mRootSignature.GetAddressOf())));
}
 
void GameApp::BuildDescriptorHeaps()
{
	//
	// Create the SRV heap.
	//
	D3D12_DESCRIPTOR_HEAP_DESC srvHeapDesc = {};
	srvHeapDesc.NumDescriptors = 11; // including 2 fonts
	srvHeapDesc.Type = D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV;
	srvHeapDesc.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE;
	ThrowIfFailed(md3dDevice->CreateDescriptorHeap(&srvHeapDesc, IID_PPV_ARGS(&mSrvDescriptorHeap)));

	//
	// Fill out the heap with actual descriptors.
	//
	CD3DX12_CPU_DESCRIPTOR_HANDLE hCpuDescriptor(mSrvDescriptorHeap->GetCPUDescriptorHandleForHeapStart());
	CD3DX12_GPU_DESCRIPTOR_HANDLE hGpuDescriptor(mSrvDescriptorHeap->GetGPUDescriptorHandleForHeapStart());

	/*auto bricksTex = m
	["bricksTex"]->Resource;
	auto stoneTex = mTextures["stoneTex"]->Resource;
	auto tileTex = mTextures["tileTex"]->Resource;
	auto crateTex = mTextures["crateTex"]->Resource;
	auto iceTex = mTextures["iceTex"]->Resource;
	auto grassTex = mTextures["grassTex"]->Resource;
	auto mudTex = mTextures["mudTex"]->Resource;
	auto defaultTex = mTextures["defaultTex"]->Resource;
	auto uiTex = mTextures["uiTex"]->Resource;*/

	//Brick 0
	D3D12_SHADER_RESOURCE_VIEW_DESC srvDesc = {};
	srvDesc.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
	srvDesc.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2D;
	srvDesc.Texture2D.MostDetailedMip = 0;
	srvDesc.Texture2D.ResourceMinLODClamp = 0.0f;

	Microsoft::WRL::ComPtr<ID3D12Resource> r;
	// in texture order
	for (auto& t : mTextures)
	{
		r = t.second->Resource;
		srvDesc.Format = r->GetDesc().Format;
		srvDesc.Texture2D.MipLevels = r->GetDesc().MipLevels;
		md3dDevice->CreateShaderResourceView(r.Get(), &srvDesc, hCpuDescriptor);

		hCpuDescriptor.Offset(1, mCbvSrvDescriptorSize);
		hGpuDescriptor.Offset(1, mCbvSrvDescriptorSize);
	}

	// sprites
	{
		mSpriteManager.Init(md3dDevice.Get(), mCommandQueue.Get(), mCbvSrvDescriptorSize, mBackBufferFormat, mDepthStencilFormat, hCpuDescriptor,hGpuDescriptor);
	}

}

void GameApp::BuildShadersAndInputLayout()
{
	const D3D_SHADER_MACRO alphaTestDefines[] =
	{
		"ALPHA_TEST", "1",
		NULL, NULL
	};

	mShaders["standardVS"] = d3dUtil::CompileShader(L"Shaders\\Default.hlsl", nullptr, "VS", "vs_5_1");
	mShaders["opaquePS"] = d3dUtil::CompileShader(L"Shaders\\Default.hlsl", nullptr, "PS", "ps_5_1");

	mInputLayout =
	{
		{ "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0 },
		{ "NORMAL", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 12, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0 },
		{ "TEXCOORD", 0, DXGI_FORMAT_R32G32_FLOAT, 0, 24, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0 },
	};
}

void GameApp::BuildBoxGeometry()
{
	GeometryGenerator geoGen;
	GeometryGenerator::MeshData box = geoGen.CreateBox(1.0f, 1.0f, 1.0f, 3);

	std::vector<Vertex> vertices(box.Vertices.size());
	for (size_t i = 0; i < box.Vertices.size(); ++i)
	{
		auto& p = box.Vertices[i].Position;
		vertices[i].Pos = p;
		vertices[i].Normal = box.Vertices[i].Normal;
		vertices[i].TexC = box.Vertices[i].TexC;
	}

	const UINT vbByteSize = (UINT)vertices.size() * sizeof(Vertex);

	std::vector<std::uint16_t> indices = box.GetIndices16();
	const UINT ibByteSize = (UINT)indices.size() * sizeof(std::uint16_t);

	auto geo = std::make_unique<MeshGeometry>();
	geo->Name = "boxGeo";

	ThrowIfFailed(D3DCreateBlob(vbByteSize, &geo->VertexBufferCPU));
	CopyMemory(geo->VertexBufferCPU->GetBufferPointer(), vertices.data(), vbByteSize);

	ThrowIfFailed(D3DCreateBlob(ibByteSize, &geo->IndexBufferCPU));
	CopyMemory(geo->IndexBufferCPU->GetBufferPointer(), indices.data(), ibByteSize);

	geo->VertexBufferGPU = d3dUtil::CreateDefaultBuffer(md3dDevice.Get(),
		mCommandList.Get(), vertices.data(), vbByteSize, geo->VertexBufferUploader);

	geo->IndexBufferGPU = d3dUtil::CreateDefaultBuffer(md3dDevice.Get(),
		mCommandList.Get(), indices.data(), ibByteSize, geo->IndexBufferUploader);

	geo->VertexByteStride = sizeof(Vertex);
	geo->VertexBufferByteSize = vbByteSize;
	geo->IndexFormat = DXGI_FORMAT_R16_UINT;
	geo->IndexBufferByteSize = ibByteSize;

	SubmeshGeometry submesh;
	submesh.IndexCount = (UINT)indices.size();
	submesh.StartIndexLocation = 0;
	submesh.BaseVertexLocation = 0;

	geo->DrawArgs["box"] = submesh;

	mGeometries["boxGeo"] = std::move(geo);
}

void GameApp::BuildObjGeometry(const std::string& filepath, const std::string& meshName, const std::string& subMeshName)
{
	struct VertexConversion
	{
		Vertex operator()(const objl::Vertex& v)
		{
			return
			{
				{v.Position.X,v.Position.Y,v.Position.Z},
				{v.Normal.X,v.Normal.Y,v.Normal.Z},
				{v.TextureCoordinate.X,-v.TextureCoordinate.Y} // flipped Y coor from blender
			};
		}
	};

	struct IndexConversion
	{
		int operator()(const unsigned int & i)
		{
			return static_cast<int>(i);
		}
	};

	struct MeshConvertion
	{
		std::vector<Vertex> vertices;
		std::vector<std::int32_t> indices;

		void operator()(const objl::Mesh& mesh)
		{
			std::transform(mesh.Vertices.begin(), mesh.Vertices.end(), std::back_inserter(vertices), VertexConversion());
			std::transform(mesh.Indices.begin(), mesh.Indices.end(), std::back_inserter(indices), IndexConversion());
		}
	};

	objl::Loader loader;

	bool loadout = loader.LoadFile(filepath);
	
	if (loadout)
	{
		MeshConvertion meshConvert;

		// converts vertex and index formats from objl to local
		std::for_each(loader.LoadedMeshes.begin(), loader.LoadedMeshes.end(), std::ref(meshConvert));

		const UINT vbByteSize = (UINT)meshConvert.vertices.size() * sizeof(Vertex);
		const UINT ibByteSize = (UINT)meshConvert.indices.size() * sizeof(std::int32_t);

		auto geo = std::make_unique<MeshGeometry>();
		geo->Name = meshName;

		ThrowIfFailed(D3DCreateBlob(vbByteSize, &geo->VertexBufferCPU));
		CopyMemory(geo->VertexBufferCPU->GetBufferPointer(), meshConvert.vertices.data(), vbByteSize);

		ThrowIfFailed(D3DCreateBlob(ibByteSize, &geo->IndexBufferCPU));
		CopyMemory(geo->IndexBufferCPU->GetBufferPointer(), meshConvert.indices.data(), ibByteSize);

		geo->VertexBufferGPU = d3dUtil::CreateDefaultBuffer(md3dDevice.Get(),
			mCommandList.Get(), meshConvert.vertices.data(), vbByteSize, geo->VertexBufferUploader);

		geo->IndexBufferGPU = d3dUtil::CreateDefaultBuffer(md3dDevice.Get(),
			mCommandList.Get(), meshConvert.indices.data(), ibByteSize, geo->IndexBufferUploader);

		geo->VertexByteStride = sizeof(Vertex);
		geo->VertexBufferByteSize = vbByteSize;
		geo->IndexFormat = DXGI_FORMAT_R32_UINT;
		geo->IndexBufferByteSize = ibByteSize;

		SubmeshGeometry submesh;
		submesh.IndexCount = (UINT)meshConvert.indices.size();
		submesh.StartIndexLocation = 0;
		submesh.BaseVertexLocation = 0;

		geo->DrawArgs[subMeshName] = submesh; 

		mGeometries[geo->Name] = std::move(geo);
	}
	else
	{
		//file not found
		assert(false);
	}

}

void GameApp::BuildTileGeometry()
{
	GeometryGenerator geoGen;
	GeometryGenerator::MeshData tile = geoGen.CreateTile(1.0f, 1.0f, 1.0f, 1);

	std::vector<Vertex> vertices(tile.Vertices.size());
	for (size_t i = 0; i < tile.Vertices.size(); ++i)
	{
		auto& p = tile.Vertices[i].Position;
		vertices[i].Pos = p;
		vertices[i].Normal = tile.Vertices[i].Normal;
		vertices[i].TexC = tile.Vertices[i].TexC;
	}

	const UINT vbByteSize = (UINT)vertices.size() * sizeof(Vertex);

	std::vector<std::uint16_t> indices = tile.GetIndices16();
	const UINT ibByteSize = (UINT)indices.size() * sizeof(std::uint16_t);

	auto geo = std::make_unique<MeshGeometry>();
	geo->Name = "tileGeo";

	ThrowIfFailed(D3DCreateBlob(vbByteSize, &geo->VertexBufferCPU));
	CopyMemory(geo->VertexBufferCPU->GetBufferPointer(), vertices.data(), vbByteSize);

	ThrowIfFailed(D3DCreateBlob(ibByteSize, &geo->IndexBufferCPU));
	CopyMemory(geo->IndexBufferCPU->GetBufferPointer(), indices.data(), ibByteSize);

	geo->VertexBufferGPU = d3dUtil::CreateDefaultBuffer(md3dDevice.Get(),
		mCommandList.Get(), vertices.data(), vbByteSize, geo->VertexBufferUploader);

	geo->IndexBufferGPU = d3dUtil::CreateDefaultBuffer(md3dDevice.Get(),
		mCommandList.Get(), indices.data(), ibByteSize, geo->IndexBufferUploader);

	geo->VertexByteStride = sizeof(Vertex);
	geo->VertexBufferByteSize = vbByteSize;
	geo->IndexFormat = DXGI_FORMAT_R16_UINT;
	geo->IndexBufferByteSize = ibByteSize;

	SubmeshGeometry submesh;
	submesh.IndexCount = (UINT)indices.size();
	submesh.StartIndexLocation = 0;
	submesh.BaseVertexLocation = 0;

	geo->DrawArgs["tile"] = submesh;

	mGeometries[geo->Name] = std::move(geo);
}

void GameApp::BuildSwordGeometry()
{
	GeometryGenerator geoGen;
	GeometryGenerator::MeshData sword = geoGen.CreateSword(1.0f, 1.0f, 1.0f, 3);

	std::vector<Vertex> vertices(sword.Vertices.size());
	for (size_t i = 0; i < sword.Vertices.size(); ++i)
	{
		auto& p = sword.Vertices[i].Position;
		vertices[i].Pos = p;
		vertices[i].Normal = sword.Vertices[i].Normal;
		vertices[i].TexC = sword.Vertices[i].TexC;
	}

	const UINT vbByteSize = (UINT)vertices.size() * sizeof(Vertex);

	std::vector<std::uint16_t> indices = sword.GetIndices16();
	const UINT ibByteSize = (UINT)indices.size() * sizeof(std::uint16_t);

	auto geo = std::make_unique<MeshGeometry>();
	geo->Name = "swordGeo";

	ThrowIfFailed(D3DCreateBlob(vbByteSize, &geo->VertexBufferCPU));
	CopyMemory(geo->VertexBufferCPU->GetBufferPointer(), vertices.data(), vbByteSize);

	ThrowIfFailed(D3DCreateBlob(ibByteSize, &geo->IndexBufferCPU));
	CopyMemory(geo->IndexBufferCPU->GetBufferPointer(), indices.data(), ibByteSize);

	geo->VertexBufferGPU = d3dUtil::CreateDefaultBuffer(md3dDevice.Get(),
		mCommandList.Get(), vertices.data(), vbByteSize, geo->VertexBufferUploader);

	geo->IndexBufferGPU = d3dUtil::CreateDefaultBuffer(md3dDevice.Get(),
		mCommandList.Get(), indices.data(), ibByteSize, geo->IndexBufferUploader);

	geo->VertexByteStride = sizeof(Vertex);
	geo->VertexBufferByteSize = vbByteSize;
	geo->IndexFormat = DXGI_FORMAT_R16_UINT;
	geo->IndexBufferByteSize = ibByteSize;

	SubmeshGeometry submesh;
	submesh.IndexCount = (UINT)indices.size();
	submesh.StartIndexLocation = 0;
	submesh.BaseVertexLocation = 0;

	geo->DrawArgs["sword"] = submesh;

	mGeometries[geo->Name] = std::move(geo);
}


void GameApp::BuildPlayerGeometry()
{
	GeometryGenerator geoGen;
	GeometryGenerator::MeshData player = geoGen.CreatePlayer(6.0f, 6.0f, 6.0f, 3);

	std::vector<Vertex> vertices(player.Vertices.size());
	for (size_t i = 0; i < player.Vertices.size(); ++i)
	{
		auto& p = player.Vertices[i].Position;
		vertices[i].Pos = p;
		vertices[i].Normal = player.Vertices[i].Normal;
		vertices[i].TexC = player.Vertices[i].TexC;
	}

	const UINT vbByteSize = (UINT)vertices.size() * sizeof(Vertex);

	std::vector<std::uint16_t> indices = player.GetIndices16();
	const UINT ibByteSize = (UINT)indices.size() * sizeof(std::uint16_t);

	auto geo = std::make_unique<MeshGeometry>();
	geo->Name = "playerGeo";

	ThrowIfFailed(D3DCreateBlob(vbByteSize, &geo->VertexBufferCPU));
	CopyMemory(geo->VertexBufferCPU->GetBufferPointer(), vertices.data(), vbByteSize);

	ThrowIfFailed(D3DCreateBlob(ibByteSize, &geo->IndexBufferCPU));
	CopyMemory(geo->IndexBufferCPU->GetBufferPointer(), indices.data(), ibByteSize);

	geo->VertexBufferGPU = d3dUtil::CreateDefaultBuffer(md3dDevice.Get(),
		mCommandList.Get(), vertices.data(), vbByteSize, geo->VertexBufferUploader);

	geo->IndexBufferGPU = d3dUtil::CreateDefaultBuffer(md3dDevice.Get(),
		mCommandList.Get(), indices.data(), ibByteSize, geo->IndexBufferUploader);

	geo->VertexByteStride = sizeof(Vertex);
	geo->VertexBufferByteSize = vbByteSize;
	geo->IndexFormat = DXGI_FORMAT_R16_UINT;
	geo->IndexBufferByteSize = ibByteSize;

	SubmeshGeometry submesh;
	submesh.IndexCount = (UINT)indices.size();
	submesh.StartIndexLocation = 0;
	submesh.BaseVertexLocation = 0;

	geo->DrawArgs["player"] = submesh;

	mGeometries[geo->Name] = std::move(geo);
}

void GameApp::BuildPSOs()
{
	D3D12_GRAPHICS_PIPELINE_STATE_DESC opaquePsoDesc;

	//
	// PSO for opaque objects.
	//
	ZeroMemory(&opaquePsoDesc, sizeof(D3D12_GRAPHICS_PIPELINE_STATE_DESC));
	opaquePsoDesc.InputLayout = { mInputLayout.data(), (UINT)mInputLayout.size() };
	opaquePsoDesc.pRootSignature = mRootSignature.Get();
	opaquePsoDesc.VS =
	{
		reinterpret_cast<BYTE*>(mShaders["standardVS"]->GetBufferPointer()),
		mShaders["standardVS"]->GetBufferSize()
	};
	opaquePsoDesc.PS =
	{
		reinterpret_cast<BYTE*>(mShaders["opaquePS"]->GetBufferPointer()),
		mShaders["opaquePS"]->GetBufferSize()
	};
	opaquePsoDesc.RasterizerState = CD3DX12_RASTERIZER_DESC(D3D12_DEFAULT);
	opaquePsoDesc.BlendState = CD3DX12_BLEND_DESC(D3D12_DEFAULT);
	opaquePsoDesc.DepthStencilState = CD3DX12_DEPTH_STENCIL_DESC(D3D12_DEFAULT);
	opaquePsoDesc.SampleMask = UINT_MAX;
	opaquePsoDesc.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
	opaquePsoDesc.NumRenderTargets = 1;
	opaquePsoDesc.RTVFormats[0] = mBackBufferFormat;
	opaquePsoDesc.SampleDesc.Count = m4xMsaaState ? 4 : 1;
	opaquePsoDesc.SampleDesc.Quality = m4xMsaaState ? (m4xMsaaQuality - 1) : 0;
	opaquePsoDesc.DSVFormat = mDepthStencilFormat;
	ThrowIfFailed(md3dDevice->CreateGraphicsPipelineState(&opaquePsoDesc, IID_PPV_ARGS(&mPSOs["opaque"])));
}

void GameApp::BuildFrameResources()
{
	for (int i = 0; i < gNumFrameResources; ++i)
	{
		mFrameResources.push_back(std::make_unique<FrameResource>(md3dDevice.Get(),
			1, mInstanceCount, (UINT)mMaterials.size()));
	}
}

void GameApp::BuildMaterial(int& matCBIndex, int texSrvHeapIndex, const std::string & name, float roughness, const DirectX::XMFLOAT4 & diffuseAlbedo, const DirectX::XMFLOAT3 & fresnel)
{

	auto material = std::make_unique<Material>();
	material->Name = name;
	material->MatCBIndex = matCBIndex;
	material->DiffuseSrvHeapIndex = texSrvHeapIndex;
	material->DiffuseAlbedo = diffuseAlbedo;
	material->FresnelR0 = fresnel;
	material->Roughness = roughness;

	mMaterials[material->Name] = std::move(material);

	++matCBIndex; //increments for next material

}

void GameApp::BuildMaterials()
{
	int matIndex = 0;
	mMaterials.reserve(20);

	// Material names are same as texture names
	std::for_each(mTextures.begin(), mTextures.end(), [&](auto& p) 
	{
		BuildMaterial(matIndex, matIndex, p.first);
	});
}

std::unique_ptr<RenderItem> GameApp::BuildRenderItem(UINT& objCBindex, const std::string& geoName, const std::string& subGeoName)
{

	auto rItem = std::make_unique<RenderItem>();
	rItem->World = MathHelper::Identity4x4();
	rItem->ObjCBIndex = objCBindex;
	rItem->InstanceCount = 0;
	rItem->Geo = mGeometries[geoName].get();
	rItem->PrimitiveType = D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST;
	rItem->IndexCount = rItem->Geo->DrawArgs  [subGeoName].IndexCount;
	rItem->StartIndexLocation = rItem->Geo->DrawArgs[subGeoName].StartIndexLocation;
	rItem->BaseVertexLocation = rItem->Geo->DrawArgs[subGeoName].BaseVertexLocation;

	++objCBindex;

	return std::move(rItem);
}

void GameApp::BuildRenderItems()
{
	// Generate instance data.

	UINT objCbIndex = 0;

	// Every rItem needs an instance buffer
	// Change GC::NUM_DIFF_RENDER_OBJS to reflect this. 
	mAllRitems["Tiles"] = BuildRenderItem(objCbIndex, "floorTileGeo", "floorTile");
	mAllRitems["Weapon"] = BuildRenderItem(objCbIndex, "tempSwordGeo", "tempSword");
	mAllRitems["Player"] = BuildRenderItem(objCbIndex, "tempPlayerGeo", "tempPlayer");
	mAllRitems["Enemy"] = BuildRenderItem(objCbIndex, "tempEnemyGeo", "tempEnemy");
	mAllRitems["Trader"] = BuildRenderItem(objCbIndex, "traderGeo", "trader");
	mAllRitems["Potato"] = BuildRenderItem(objCbIndex, "potatoGeo", "potato");
	mAllRitems["particle"] = BuildRenderItem(objCbIndex, "particle", "particle");
	mAllRitems["CarBody"] = BuildRenderItem(objCbIndex, "carBodyGeo", "carBody");
	mAllRitems["CarTire"] = BuildRenderItem(objCbIndex, "carTireGeo", "carTire");
	mAllRitems["EnemyGhoul"] = BuildRenderItem(objCbIndex, "enemyGhoulGeo", "enemyGhoul");
	mAllRitems["Building"] = BuildRenderItem(objCbIndex, "buildingGeo", "building");

	// All the render items are opaque.
	for (auto& e : mAllRitems)
		mOpaqueRitems.push_back(e.second.get());

	// check theres an instance buffer for each render item. 
	assert(mOpaqueRitems.size() == (sizeof(mCurrFrameResource->InstanceBuffer) / sizeof(mCurrFrameResource->InstanceBuffer[0])));
}



void GameApp::DrawRenderItems(ID3D12GraphicsCommandList* cmdList, const std::vector<RenderItem*>& ritems)
{

	// For each render item...
	for (size_t i = 0; i < ritems.size(); ++i)
	{
		auto ri = ritems[i];

		cmdList->IASetVertexBuffers(0, 1, &ri->Geo->VertexBufferView());
		cmdList->IASetIndexBuffer(&ri->Geo->IndexBufferView());
		cmdList->IASetPrimitiveTopology(ri->PrimitiveType);

		// Set the instance buffer to use for this render-item.  For structured buffers, we can bypass 
		// the heap and set as a root descriptor.
		auto instanceBuffer = mCurrFrameResource->InstanceBuffer[i]->Resource();
		mCommandList->SetGraphicsRootShaderResourceView(0, instanceBuffer->GetGPUVirtualAddress());

		cmdList->DrawIndexedInstanced(ri->IndexCount, ri->InstanceCount, ri->StartIndexLocation, ri->BaseVertexLocation, 0);
	}
}

CD3DX12_GPU_DESCRIPTOR_HANDLE GameApp::GetSpriteGpuDescHandle(const std::string & textureName)
{
	// get gpu start
	CD3DX12_GPU_DESCRIPTOR_HANDLE hGpuDescriptor(mSrvDescriptorHeap->GetGPUDescriptorHandleForHeapStart());

	//find offset distance
	const int distance = (int)std::distance(mTextures.begin(), mTextures.find(textureName));

	//offset desc
	hGpuDescriptor.Offset(distance, mCbvSrvUavDescriptorSize);

	return hGpuDescriptor;
}

std::array<const CD3DX12_STATIC_SAMPLER_DESC, 6> GameApp::GetStaticSamplers()
{
	// Applications usually only need a handful of samplers.  So just define them all up front
	// and keep them available as part of the root signature.  

	const CD3DX12_STATIC_SAMPLER_DESC pointWrap(
		0, // shaderRegister
		D3D12_FILTER_MIN_MAG_MIP_POINT, // filter
		D3D12_TEXTURE_ADDRESS_MODE_WRAP,  // addressU
		D3D12_TEXTURE_ADDRESS_MODE_WRAP,  // addressV
		D3D12_TEXTURE_ADDRESS_MODE_WRAP); // addressW

	const CD3DX12_STATIC_SAMPLER_DESC pointClamp(
		1, // shaderRegister
		D3D12_FILTER_MIN_MAG_MIP_POINT, // filter
		D3D12_TEXTURE_ADDRESS_MODE_CLAMP,  // addressU
		D3D12_TEXTURE_ADDRESS_MODE_CLAMP,  // addressV
		D3D12_TEXTURE_ADDRESS_MODE_CLAMP); // addressW

	const CD3DX12_STATIC_SAMPLER_DESC linearWrap(
		2, // shaderRegister
		D3D12_FILTER_MIN_MAG_MIP_LINEAR, // filter
		D3D12_TEXTURE_ADDRESS_MODE_WRAP,  // addressU
		D3D12_TEXTURE_ADDRESS_MODE_WRAP,  // addressV
		D3D12_TEXTURE_ADDRESS_MODE_WRAP); // addressW

	const CD3DX12_STATIC_SAMPLER_DESC linearClamp(
		3, // shaderRegister
		D3D12_FILTER_MIN_MAG_MIP_LINEAR, // filter
		D3D12_TEXTURE_ADDRESS_MODE_CLAMP,  // addressU
		D3D12_TEXTURE_ADDRESS_MODE_CLAMP,  // addressV
		D3D12_TEXTURE_ADDRESS_MODE_CLAMP); // addressW

	const CD3DX12_STATIC_SAMPLER_DESC anisotropicWrap(
		4, // shaderRegister
		D3D12_FILTER_ANISOTROPIC, // filter
		D3D12_TEXTURE_ADDRESS_MODE_WRAP,  // addressU
		D3D12_TEXTURE_ADDRESS_MODE_WRAP,  // addressV
		D3D12_TEXTURE_ADDRESS_MODE_WRAP,  // addressW
		0.0f,                             // mipLODBias
		8);                               // maxAnisotropy

	const CD3DX12_STATIC_SAMPLER_DESC anisotropicClamp(
		5, // shaderRegister
		D3D12_FILTER_ANISOTROPIC, // filter
		D3D12_TEXTURE_ADDRESS_MODE_CLAMP,  // addressU
		D3D12_TEXTURE_ADDRESS_MODE_CLAMP,  // addressV
		D3D12_TEXTURE_ADDRESS_MODE_CLAMP,  // addressW
		0.0f,                              // mipLODBias
		8);                                // maxAnisotropy

	return {
		pointWrap, pointClamp,
		linearWrap, linearClamp,
		anisotropicWrap, anisotropicClamp };
}

const std::string& GameApp::GetStoryText() 
{
	// defaults if out of range
	if (mStoryIndex >= GC::STORY_TEXT_SIZE)
	{
		mStoryIndex = 0;
	}

	return GC::STORY_TEXT[mStoryIndex];
}

const std::string& GameApp::GetTutorialText()
{
	if (mTutorialText == "")
		assert(mTutorialText == "");

	return mTutorialText;
}

PassConstants * GameApp::GetMainPassCB()
{
	return &mMainPassCB;
};
