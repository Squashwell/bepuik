#ifndef PERMUTATIONMAPPER_HPP
#define PERMUTATIONMAPPER_HPP

namespace BEPUik
{
    /// <summary>
    /// Maps indices to permuted versions of the indices.
    /// </summary>
    class PermutationMapper
    {
    private:
        __INT64_TYPE__ permutationIndex;
        __INT64_TYPE__ currentOffset;
        __INT64_TYPE__ currentPrime;
//        static __INT64_TYPE__ primes[];
//        static __INT64_TYPE__ primesLength;
    public:
        /// <summary>
        /// Constructs a new permutation mapper.
        /// </summary>
        PermutationMapper();

        /// <summary>
        /// Gets or sets the permutation index used by the solver.  If the simulation is restarting from a given frame,
        /// setting this index to be consistent is required for deterministic results.
        /// </summary>
        void SetPermutationIndex(__INT64_TYPE__ value);
        __INT64_TYPE__ GetPermutationIndex();

        /// <summary>
        /// Gets a remapped index.
        /// </summary>
        /// <param name="index">Original index of an element in the set to be redirected to a shuffled position.</param>
        /// <param name="setSize">Size of the set being permuted. Must be smaller than 350000041.</param>
        /// <returns>The remapped index.</returns>
        __INT64_TYPE__ GetMappedIndex(__INT64_TYPE__ index, int setSize);
    };
    

}


#endif // PERMUTATIONMAPPER_HPP
