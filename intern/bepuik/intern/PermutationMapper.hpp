#ifndef PERMUTATIONMAPPER_HPP
#define PERMUTATIONMAPPER_HPP

#include "BLI_sys_types.h"

namespace BEPUik
{
    /// <summary>
    /// Maps indices to permuted versions of the indices.
    /// </summary>
    class PermutationMapper
    {
    private:
		int64_t permutationIndex;
		int64_t currentOffset;
		int64_t currentPrime;

    public:
        /// <summary>
        /// Constructs a new permutation mapper.
        /// </summary>
        PermutationMapper();

        /// <summary>
        /// Gets or sets the permutation index used by the solver.  If the simulation is restarting from a given frame,
        /// setting this index to be consistent is required for deterministic results.
        /// </summary>
		void SetPermutationIndex(int64_t value);
		int64_t GetPermutationIndex();

        /// <summary>
        /// Gets a remapped index.
        /// </summary>
        /// <param name="index">Original index of an element in the set to be redirected to a shuffled position.</param>
        /// <param name="setSize">Size of the set being permuted. Must be smaller than 350000041.</param>
        /// <returns>The remapped index.</returns>
		int64_t GetMappedIndex(int64_t index, int setSize);
    };
    

}


#endif // PERMUTATIONMAPPER_HPP
