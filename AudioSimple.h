#pragma once

#include "Audio.h"
#include <string>

#include <unordered_map>
#include <map>
#include <array>

class GameTimer;

//Simple Audio Demo
class AudioSimple
{
	typedef std::pair< std::unique_ptr<DirectX::SoundEffect>, std::unique_ptr<DirectX::SoundEffectInstance>> SoundPair;

	std::unique_ptr<DirectX::AudioEngine> mAudioEngine;			// Globals for sound effects and instances assigned to it
	std::unordered_map<std::string, SoundPair>  mSounds;		// Sounds assigned a string name

	DirectX::AudioEmitter mEmitter; 
	DirectX::AudioListener mListener; // camera properties passed in through Update()
	X3DAUDIO_CONE mCone;
public:
	~AudioSimple();

	void Init();
	void Update(const GameTimer& gt, const DirectX::XMFLOAT3& camPos , const DirectX::XMFLOAT3& camForward, const DirectX::XMFLOAT3& camUp);
	void Load(const std::string& name, const std::wstring& filename);
	void Play(const std::string& name, bool loop = false, float volume = 1.0f,float pitch = 0.0f,float pan = 0.0f);
	void Suspend();
	void Resume();
	void SetReverbRandom();
};

/*
      SoundEngine
        |    |
    -----    -----
   |              |
SfxEngine    MusicEngine


GameAudio
	vector<SoundEngine>

*/

enum AUDIO_ENGINE_TYPE { SFX, MUSIC };


class SoundEngine
{
protected:
	std::unique_ptr<DirectX::AudioEngine> mAudioEngine;
	std::map< std::string, std::unique_ptr<DirectX::SoundEffect>> mSounds;	// Load sounds and assigned to engine
	std::list<std::unique_ptr<DirectX::SoundEffectInstance>> mCache;		//Plays instances of sounds
	size_t mCacheLimit;
	DirectX::AudioListener* pListener;
	float mNormalisedVolume = 1.0f;
	float mFadeInSecs = 3.0f; 
	DirectX::SOUND_EFFECT_INSTANCE_FLAGS GetInstanceFlags(DirectX::AudioEmitter* emitter); //Flags set based on if emitter provided
	AUDIO_ENGINE_TYPE mType;
public:
	SoundEngine(DirectX::AudioListener* listener);
	virtual void Init() = 0;
	virtual void Update(const GameTimer& gt) = 0;
	virtual void Play(const std::string& soundName, DirectX::AudioEmitter* emitter = nullptr, bool loop = false, float volume = 1.0f, float pitch = 0.0f, float pan = 0.0f) = 0;
	void Pause();
	void Resume();
	void SetFade(float secs);
	void SetCacheLimit(size_t limit);
	void LoadSound(const std::string& soundName, const std::wstring& filename);
	virtual void ForceAudio(bool force) = 0;
	AUDIO_ENGINE_TYPE GetType();
	void SetEngineVolume(float volume, bool increment = false);
	const float GetEngineVolume();
};

class SfxEngine : public SoundEngine
{
	// Forces new instance to play when cache is full by removing oldest instance.  
	bool mForceAudio; 
public:
	SfxEngine(DirectX::AudioListener* listener);
	void Init() override;
	void Update(const GameTimer& gt) override;
	void Play(const std::string& soundName, DirectX::AudioEmitter* emitter = nullptr, bool loop = false, float volume = 1.0f, float pitch = 0.0f, float pan = 0.0f) override; // push back mCache
	void ForceAudio(bool force) override;
};

class MusicEngine : public SoundEngine
{
private:
	// To comparing if track is already playing
	std::string mFrontAudio = "";
	//Swaps front to back
	void SwapCache();
public:
	MusicEngine(DirectX::AudioListener* listener);
	void Init() override;
	void Update(const GameTimer& gt) override;
	void Play(const std::string& soundName, DirectX::AudioEmitter* emitter = nullptr,  bool loop = false, float volume = 1.0f, float pitch = 0.0f, float pan = 0.0f) override;
	void ForceAudio(bool force) override {};
};

class GameAudio
{
private:
	std::map<std::string, std::unique_ptr<SoundEngine>> mEngines;
	std::map<std::string, std::string> mkeys; //audioname and enginename
	float masterVolume;
	DirectX::AudioListener mListener; 
	X3DAUDIO_CONE mCone; // for 3D sound. Default set in Init()
	bool ValidEngine(const std::string& name);
	bool ValidKeys(const std::string& engineName, const std::string& soundName);
public:

	~GameAudio();
	void Init();
	// Creates specialised engines for audio playback
	void CreateEngine(const std::string& name, const AUDIO_ENGINE_TYPE& type);
	void Update(const GameTimer& gt, const DirectX::XMFLOAT3& camPos, const DirectX::XMFLOAT3& camForward, const DirectX::XMFLOAT3& camUp);
	// Plays audio. Engine name not required. Supplying AudioEmitter* applies 3D sound. 
	void Play(const std::string& soundName, DirectX::AudioEmitter* emitter = nullptr, bool loop = false, float volume = 1.0f, float pitch = 0.0f, float pan = 0.0f);
	// Pause specific engine
	void Pause(const std::string& engineName);
	// Resume specific engine
	void Resume(const std::string& engineName);
	void PauseAll();
	void ResumeAll();
	void LoadSound(const std::string& engineName, const std::string& soundName, const std::wstring& filename);
	// How many audio clips can play at once per engine. SFX engine only. Minimum of 1. Music engines have cache size of 2.
	void SetCacheSize(const std::string& name, size_t limit);
	// Forces new instance to play when cache is full by removing oldest instance. SFX Engine only
	void ForceAudio(const std::string& name, bool force);
	// Fade effect for music audio engines
	void SetFade(const std::string& name, float secs);
	void SetEngineVolume(const std::string& engineName, float volume, bool increment = false);
	// Set volume of all engines
	void SetVolume(float volume, bool increment = false);


};


