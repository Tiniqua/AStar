#include "AudioSImple.h"
#include "Common/GameTimer.h"
#include <string>
#include <algorithm>
#include <iterator>

AudioSimple::~AudioSimple()
{
	if (mAudioEngine)
	{
		mAudioEngine->Suspend();
	}
}

void AudioSimple::Init()
{
	HRESULT hr = CoInitializeEx(nullptr, COINIT_MULTITHREADED);
	if (FAILED(hr))
	{
		assert(false);
	}

	DirectX::AUDIO_ENGINE_FLAGS eflags = DirectX::AudioEngine_Default | DirectX::AudioEngine_EnvironmentalReverb | DirectX::AudioEngine_ReverbUseFilters;


#ifdef _DEBUG
	eflags =  eflags | DirectX::AudioEngine_Debug;
#endif
	//Setup engine
	mAudioEngine = std::make_unique<DirectX::AudioEngine>(eflags,nullptr,nullptr, AudioCategory_GameEffects);

	// Cone setup for directional sound
	mCone = X3DAudioDefault_DirectionalCone; //// { X3DAUDIO_PI / 2, X3DAUDIO_PI, 1.0f, 0.708f, 0.0f, 0.25f, 0.708f, 1.0f };
	// Override default directional cone properties
	mCone.InnerAngle = X3DAUDIO_PI / 4;
	mCone.OuterAngle = X3DAUDIO_PI / 2;
	mCone.InnerVolume = 2.0f;
	mCone.OuterVolume = 0.0f; // can't hear outside of OuterAngle at 0.0f
	//Assign cone to listener
	mListener.pCone = &mCone;


}

void AudioSimple::Update(const GameTimer& gt, const DirectX::XMFLOAT3& camPos, const DirectX::XMFLOAT3& camForward, const DirectX::XMFLOAT3& camUp)
{
	if (mAudioEngine->Update())
	{
		//Passes camera properties for 3D sound
		mListener.SetPosition(camPos);
		mListener.SetOrientation(camForward, camUp);

		//emitter example
		mEmitter.SetPosition(DirectX::XMFLOAT3(cosf(gt.TotalTime())*0.0f, 0.0f, 1.0f));

	}
	else
	{
		// No audio device is active
		if (mAudioEngine->IsCriticalError())
		{
			assert(false);
		}
	}
}

void AudioSimple::Load(const std::string& name, const std::wstring& filename)
{
	SoundPair soundPair;
	soundPair.first = std::make_unique<DirectX::SoundEffect>(mAudioEngine.get(), filename.c_str() );
	assert(soundPair.first);
	
	mSounds[name] = std::move(soundPair);
	
}

void AudioSimple::Play(const std::string& name, bool loop, float volume, float pitch , float pan )
{
	assert(mSounds[name].first);

	DirectX::SOUND_EFFECT_INSTANCE_FLAGS iflags =
		DirectX::SoundEffectInstance_Default 
		|
		DirectX::SoundEffectInstance_Use3D 
		;

	mSounds[name].second = std::move(mSounds[name].first->CreateInstance(iflags));

	mSounds[name].second->Play(loop);
	mSounds[name].second->Apply3D(mListener,mEmitter);
	mSounds[name].second->SetVolume(volume);
	//mSounds[name].second->SetPitch(pitch);
	//mSounds[name].second->SetPan(pan);
}

void AudioSimple::Suspend()
{
	mAudioEngine->Suspend();
}

void AudioSimple::Resume()
{
	mAudioEngine->Resume();
}

void AudioSimple::SetReverbRandom()
{
	mAudioEngine->SetReverb((DirectX::AUDIO_ENGINE_REVERB)(rand()% DirectX::AUDIO_ENGINE_REVERB::Reverb_MAX));
}



SfxEngine::SfxEngine(DirectX::AudioListener* listener)
	:
	SoundEngine(listener),
	mForceAudio(true)
{
	mType = AUDIO_ENGINE_TYPE::SFX;
	mCacheLimit = 1;
}

void SfxEngine::Init()
{
	
	DirectX::AUDIO_ENGINE_FLAGS eflags = DirectX::AudioEngine_Default | DirectX::AudioEngine_EnvironmentalReverb | DirectX::AudioEngine_ReverbUseFilters;

#ifdef _DEBUG
	eflags = eflags | DirectX::AudioEngine_Debug;
#endif

	//Setup engine for sfx
	mAudioEngine = std::make_unique<DirectX::AudioEngine>(eflags, nullptr, nullptr, AudioCategory_GameEffects);

}

void SfxEngine::Update(const GameTimer & gt)
{
	if (mAudioEngine->Update())
	{
		//Remove audio that has stopped
		mCache.remove_if([](const auto& c) {
			return c->GetState() == DirectX::SoundState::STOPPED;
		});
	}
	else
	{
		// No audio device is active
		if (mAudioEngine->IsCriticalError())
		{
			assert(false);
		}
	}
	
}

void SfxEngine::Play(const std::string & soundName, DirectX::AudioEmitter * emitter, bool loop, float volume, float pitch, float pan)
{
	bool play = true;

	if (mCache.size() >= max(mCacheLimit,1) )
	{
		if (mForceAudio)
			//if(mCache.size() > 0)
				mCache.pop_back();
		else
			play = false;
	}

	if (play)
	{
		assert(mSounds[soundName]);

		DirectX::SOUND_EFFECT_INSTANCE_FLAGS iflags = GetInstanceFlags(emitter);

		mCache.push_front(std::move(mSounds[soundName]->CreateInstance(iflags)));

		mCache.front()->Play(false); //no looping

		if (emitter)
		{
			//3D
			mCache.front()->Apply3D(*pListener, *emitter);
		}
		else
		{
			//Non-3D
			mCache.front()->SetVolume(volume);
			mCache.front()->SetPitch(pitch);
			mCache.front()->SetPan(pan);
		}
	}
	else
	{
		std::string str = "Warning - Unable to play '" + soundName + "' due to insufficent cache\n";
		OutputDebugStringA(str.c_str());
	}


	
	

	
}

void SfxEngine::ForceAudio(bool force)
{
	mForceAudio = force;
}



MusicEngine::MusicEngine(DirectX::AudioListener* listener)
	:
	SoundEngine(listener)
{
	mType = AUDIO_ENGINE_TYPE::MUSIC;
	mCacheLimit = 2; // Only front and back
}

void MusicEngine::Init()
{
	DirectX::AUDIO_ENGINE_FLAGS eflags = DirectX::AudioEngine_Default;

#ifdef _DEBUG
	eflags = eflags | DirectX::AudioEngine_Debug;
#endif
	//Setup engine for music and ambience
	mAudioEngine = std::make_unique<DirectX::AudioEngine>(eflags, nullptr, nullptr, AudioCategory_GameMedia);
	
	assert(mCache.size() == 0);

	std::generate_n(std::back_inserter(mCache), mCacheLimit, []() 
	{
		
		return std::unique_ptr<DirectX::SoundEffectInstance>{};
	});
	
	
	
}

void MusicEngine::Update(const GameTimer & gt)
{


	if (mAudioEngine->Update())
	{
		assert(mCache.size() == mCacheLimit);

		//Increase volume for cache
		mNormalisedVolume += gt.DeltaTime() / mFadeInSecs;

		//Limit normalised volume
		if (mNormalisedVolume > 1.0f)
			mNormalisedVolume = 1.0f;
		else if (mNormalisedVolume < 0.0f)
			mNormalisedVolume = 0.0f;

		//Apply volume
		if (mCache.front())
			mCache.front()->SetVolume(mNormalisedVolume);
		if (mCache.back())
			mCache.back()->SetVolume((1.0f - mNormalisedVolume)); //compliment

	}
	else
	{
		if (mAudioEngine->IsCriticalError())
		{
			assert(false);
		}
	}

	

}

void MusicEngine::Play(const std::string & soundName, DirectX::AudioEmitter * emitter, bool loop, float volume, float pitch, float pan)
{
	assert(mSounds[soundName]);

	// Doesn't fade into the same track
	if (soundName != mFrontAudio)
	{
		mFrontAudio = soundName;

		SwapCache(); //Current front audio swapped with last

		DirectX::SOUND_EFFECT_INSTANCE_FLAGS iflags = GetInstanceFlags(emitter);

		//New instance to first cache element
		mCache.front() = std::move(mSounds[soundName]->CreateInstance(iflags));

		mCache.front()->Play(loop);

		//Set properties
		if (emitter && pListener)
		{
			mCache.front()->Apply3D(*pListener, *emitter);
		}
		else
		{
			mCache.front()->SetVolume(mNormalisedVolume);
			mCache.front()->SetPitch(pitch);
			mCache.front()->SetPan(pan);
		}
	}

	

}

void MusicEngine::SwapCache()
{
	mCache.front().swap(mCache.back());
	mNormalisedVolume = 1.0f - mNormalisedVolume;//swap audio volume 
}



bool GameAudio::ValidEngine(const std::string & name)
{
	if (mEngines.count(name) == 1)
	{
		return true;
	}
	else
	{
		std::string str = "Error - SoundEngine '" + name + "' does not exist.\n";
		OutputDebugStringA(str.c_str());
		return false;
	}
}

bool GameAudio::ValidKeys(const std::string & engineName, const std::string & soundName)
{
	bool b = false;
	
	if (mkeys.count(soundName) == 1)
	{
		if (mkeys[soundName] == engineName)
		{
			b = true;
		}
		else
		{
			std::string str = "Error - Sound '" + soundName + "' is not contained within '" + engineName + "'. Contained in '" + mkeys[soundName] + "'.\n";
			OutputDebugStringA(str.c_str());
		}
	}
	else
	{
		std::string str = "Error - Sound '" + soundName + "' does not exist.\n ";
		OutputDebugStringA(str.c_str());
	}
	assert(b);
	return b;
}

GameAudio::~GameAudio()
{
	OutputDebugStringA("~GameAudio   \n");
	mEngines.clear();
}

void GameAudio::Init()
{
	HRESULT hr = CoInitializeEx(nullptr, COINIT_MULTITHREADED);
	if (FAILED(hr))
	{
		assert(false);
	}
	mCone = X3DAudioDefault_DirectionalCone;

}

void GameAudio::CreateEngine(const std::string & name, const AUDIO_ENGINE_TYPE & type)
{
	assert(name.length() > 0);

	if (!mEngines[name])
	{
		switch (type)
		{
		case SFX:	mEngines[name] = std::make_unique<SfxEngine>(&mListener); break;
		case MUSIC:	mEngines[name] = std::make_unique<MusicEngine>(&mListener); break;
		default:			assert(false);
		}

		mEngines[name]->Init();
	}
	else
	{
		assert(false);
	}
}

void GameAudio::Update(const GameTimer & gt, const DirectX::XMFLOAT3 & camPos, const DirectX::XMFLOAT3 & camForward, const DirectX::XMFLOAT3 & camUp)
{
	//Camera as listener
	mListener.SetPosition(camPos);
	mListener.SetOrientation(camForward, camUp);

	for (auto& engine : mEngines)
	{
		engine.second->Update(gt);
	}
}

void GameAudio::Play(const std::string & soundName, DirectX::AudioEmitter * emitter, bool loop, float volume, float pitch, float pan)
{
	if (mkeys.count(soundName) == 1)
	{
		mEngines[mkeys[soundName]]->Play(soundName, emitter, loop,volume,pitch,pan);
	}
	else
	{
		std::string msg = "Error - Audio cannot play: " + soundName + "\n";
		OutputDebugStringA(msg.c_str());
		assert(false);
	}

	
}

void GameAudio::Pause(const std::string & engineName)
{
	if (ValidEngine(engineName))
		mEngines[engineName]->Pause();
}

void GameAudio::Resume(const std::string & engineName)
{
	if (ValidEngine(engineName))
		mEngines[engineName]->Resume();
}

void GameAudio::PauseAll()
{
	std::for_each(mEngines.begin(), mEngines.end(), [](auto& e)
	{
		e.second->Pause();
	});
}

void GameAudio::ResumeAll()
{
	std::for_each(mEngines.begin(), mEngines.end(), [](auto& e)
	{
		e.second->Resume();
	});
}

void GameAudio::LoadSound(const std::string & engineName, const std::string & soundName, const std::wstring & filename)
{
	std::string debug = "";


	//Check sound doesnt already exist
	if (mkeys.count(soundName) == 0)
	{
		//Create an SFX engine if one doesn't exist
		if (!ValidEngine(engineName))
		{
			CreateEngine(engineName, SFX);
			OutputDebugStringA("Warning - Engine not created. Default SFX_Engine created");
		}

		//Adds keys to map
		mkeys[soundName] = engineName;

		//Add audio to engine
		mEngines[engineName]->LoadSound(soundName, filename);

		std::string debug = "Success - Audio loaded: " + soundName + "on engine " + engineName + "\n";
	}
	else
	{
		std::string debug = "Warning - Audio already loaded: " + soundName + "on engine " + engineName + "\n";
	}
	
	OutputDebugStringA(debug.c_str());
}

void GameAudio::SetCacheSize(const std::string & name, size_t limit)
{
	assert(ValidEngine(name));

	switch (mEngines[name]->GetType())
	{
	case AUDIO_ENGINE_TYPE::SFX:	mEngines[name]->SetCacheLimit(limit); break;
	case AUDIO_ENGINE_TYPE::MUSIC:	OutputDebugStringA("Warning - Cannot change cache size of music audio engines.\n"); break;

	}
}

void GameAudio::ForceAudio(const std::string & name, bool force)
{
	if (ValidEngine(name))
	{
		switch (mEngines[name]->GetType())
		{
		case AUDIO_ENGINE_TYPE::SFX:	mEngines[name]->ForceAudio(force); break;
		case AUDIO_ENGINE_TYPE::MUSIC:	OutputDebugStringA("Warning - No forcing audio for music audio engines.\n"); break;
		}
	}

}

void GameAudio::SetFade(const std::string & name, float secs)
{
	if (ValidEngine(name))
	{
		
		switch (mEngines[name]->GetType())
		{
		case AUDIO_ENGINE_TYPE::MUSIC:	mEngines[name]->SetFade(secs); break;
		case AUDIO_ENGINE_TYPE::SFX:	OutputDebugStringA("Warning - SFX audio engines do not support fading.\n"); break;
		}
		
	}
}

void GameAudio::SetEngineVolume(const std::string & engineName, float volume, bool increment)
{
	if (ValidEngine(engineName))
	{
		mEngines[engineName]->SetEngineVolume(volume, increment);
	}
}

void GameAudio::SetVolume(float volume, bool increment)
{
	for (auto& e : mEngines)
	{
		e.second->SetEngineVolume(volume, increment);
	}
}





DirectX::SOUND_EFFECT_INSTANCE_FLAGS SoundEngine::GetInstanceFlags(DirectX::AudioEmitter * emitter)
{
	DirectX::SOUND_EFFECT_INSTANCE_FLAGS iflags = DirectX::SoundEffectInstance_Default;

	//if emitter provided then 3d sound filter applied 
	if (emitter)
		iflags = iflags | DirectX::SoundEffectInstance_Use3D;

	return iflags;
}

SoundEngine::SoundEngine(DirectX::AudioListener* listener)
	:
	pListener(listener)
{

}

void SoundEngine::SetFade(float secs)
{
	mFadeInSecs = abs(secs);
}

void SoundEngine::SetCacheLimit(size_t limit)
{
	mCacheLimit = limit;
}

void SoundEngine::LoadSound(const std::string & soundName, const std::wstring & filename)
{
	//Loads a sound for the engine
	auto sound = std::make_unique<DirectX::SoundEffect>(mAudioEngine.get(), filename.c_str());
	assert(sound);
	mSounds[soundName] = std::move(sound);
}

void SoundEngine::Pause()
{
	mAudioEngine->Suspend();
}

void SoundEngine::Resume()
{
	mAudioEngine->Resume();
}

AUDIO_ENGINE_TYPE SoundEngine::GetType()
{
	return mType;
}

void SoundEngine::SetEngineVolume(float volume, bool increment)
{
	if (increment)
	{
		volume += mAudioEngine->GetMasterVolume();
	}

	// set limits
	if (volume < 0.0f)
	{
		volume = 0.0f;
	}
	else
	{
		if (volume > 1.0f)
		{
			volume = 1.0f;
		}
	}

	mAudioEngine->SetMasterVolume(volume);
}

const float SoundEngine::GetEngineVolume()
{
	return mAudioEngine->GetMasterVolume();
}


