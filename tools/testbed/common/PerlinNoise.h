#ifndef PERLIN_NOISE_H
#define PERLIN_NOISE_H

/// Class PerlinNoise
/// Code from http://stackoverflow.com/questions/4753055/perlin-noise-generation-for-terrain
class PerlinNoise {

	public:

	  // Constructor
	  PerlinNoise();
	  PerlinNoise(double _persistence, double _frequency, double _amplitude, int32_t _octaves, int32_t _randomseed);

	  // Get Height
	  double GetHeight(double x, double y) const;

	  // Get
	  double Persistence() const { return persistence; }
	  double Frequency()   const { return frequency;   }
	  double Amplitude()   const { return amplitude;   }
	  int32_t	Octaves()	 const { return octaves;	 }
	  int32_t	RandomSeed()  const { return randomseed;  }

	  // Set
	  void Set(double _persistence, double _frequency, double _amplitude, int32_t _octaves, int32_t _randomseed);

	  void SetPersistence(double _persistence) { persistence = _persistence; }
	  void SetFrequency(  double _frequency)   { frequency = _frequency;	 }
	  void SetAmplitude(  double _amplitude)   { amplitude = _amplitude;	 }
	  void SetOctaves(	int32_t	_octaves)	 { octaves = _octaves;		 }
	  void SetRandomSeed( int32_t	_randomseed)  { randomseed = _randomseed;   }

	private:

		double Total(double i, double j) const;
		double GetValue(double x, double y) const;
		double Interpolate(double x, double y, double a) const;
		double Noise(int32_t x, int32_t y) const;

		double persistence, frequency, amplitude;
		int32_t octaves, randomseed;
};

#endif
