import numpy as np
class ClassicalSubsetSumOracle:
    def __init__(self, integer_set, target_sum):
        self.integer_set = integer_set
        self.target_sum = target_sum
        self.n_elements = len(integer_set)
        self.evaluation_count = 0  # Track oracle calls
        
    def evaluate(self, subset_indices):
        """
        Main oracle evaluation function
        """
        self.evaluation_count += 1
        subset_sum = sum(self.integer_set[i] for i in subset_indices)
        return subset_sum == self.target_sum
    
    def evaluate_bitstring(self, bitstring):
        """
        Evaluate oracle using binary string representation
        """
        if len(bitstring) != self.n_elements:
            raise ValueError(f"Bitstring must be {self.n_elements} bits long")
        
        subset_indices = [i for i, bit in enumerate(bitstring) if bit == '1']
        return self.evaluate(subset_indices)
    
    def evaluate_with_details(self, subset_indices):
        """
        Oracle with detailed information about the evaluation
        """
        subset_values = [self.integer_set[i] for i in subset_indices]
        subset_sum = sum(subset_values)
        is_solution = subset_sum == self.target_sum
        
        return {
            'indices': list(subset_indices),
            'values': subset_values,
            'sum': subset_sum,
            'target': self.target_sum,
            'is_solution': is_solution,
            'error': subset_sum - self.target_sum
        }
    
    def find_all_solutions(self):
        """
        Use oracle to find all valid subsets
        """
        from itertools import combinations
        solutions = []
        
        for r in range(1, self.n_elements + 1):
            for subset_indices in combinations(range(self.n_elements), r):
                if self.evaluate(subset_indices):
                    solution_details = self.evaluate_with_details(subset_indices)
                    solutions.append(solution_details)
        
        return solutions
    
    def get_statistics(self):
        """
        Get oracle usage statistics
        """
        return {
            'total_evaluations': self.evaluation_count,
            'integer_set': self.integer_set,
            'target_sum': self.target_sum,
            'search_space_size': 2**self.n_elements - 1  # Exclude empty set
        }

# Enhanced oracle with optimization hints
class OptimizedSubsetSumOracle(ClassicalSubsetSumOracle):
    def __init__(self, integer_set, target_sum):
        super().__init__(integer_set, target_sum)
        self.precompute_hints()
    
    def precompute_hints(self):
        """
        Precompute optimization hints for faster search
        """
        self.positive_elements = [i for i, x in enumerate(self.integer_set) if x > 0]
        self.negative_elements = [i for i, x in enumerate(self.integer_set) if x < 0]
        self.zero_elements = [i for i, x in enumerate(self.integer_set) if x == 0]
        
        # Compute bounds
        self.max_possible_sum = sum(x for x in self.integer_set if x > 0)
        self.min_possible_sum = sum(x for x in self.integer_set if x < 0)
        
        # Quick feasibility check
        self.is_feasible = self.min_possible_sum <= self.target_sum <= self.max_possible_sum
        
    def quick_feasibility_check(self, subset_indices):
        """
        Quick check if subset could possibly sum to target
        """
        if not self.is_feasible:
            return False
            
        subset_values = [self.integer_set[i] for i in subset_indices]
        current_sum = sum(subset_values)
        
        # Check if we can reach target with remaining elements
        remaining_indices = set(range(self.n_elements)) - set(subset_indices)
        remaining_positive = sum(self.integer_set[i] for i in remaining_indices if self.integer_set[i] > 0)
        remaining_negative = sum(self.integer_set[i] for i in remaining_indices if self.integer_set[i] < 0)
        
        min_reachable = current_sum + remaining_negative
        max_reachable = current_sum + remaining_positive
        
        return min_reachable <= self.target_sum <= max_reachable
    
    def evaluate_with_pruning(self, subset_indices):
        """
        Oracle evaluation with pruning for optimization
        """
        # Quick feasibility check first
        if not self.quick_feasibility_check(subset_indices):
            return False
            
        return self.evaluate(subset_indices)
        
# Probability-based oracle (mimics quantum behavior)
class ProbabilisticOracle(ClassicalSubsetSumOracle):
    def __init__(self, integer_set, target_sum, bias_strength=0.1):
        super().__init__(integer_set, target_sum)
        self.bias_strength = bias_strength
        import random
        self.random = random.Random(42)  # Fixed seed for reproducibility
    
    def probabilistic_evaluate(self, subset_indices):
        """
        Oracle that returns probabilistic results (mimics quantum behavior)
        """
        # Get true oracle result
        true_result = self.evaluate(subset_indices)
        
        # Add bias toward correct solutions
        if true_result:
            # Correct solutions have higher probability of being returned as True
            return self.random.random() > (0.1 * self.bias_strength)
        else:
            # Incorrect solutions have small chance of false positive
            return self.random.random() < (0.05 * self.bias_strength)
    
    def biased_search(self, max_evaluations=1000):
        """
        Search using probabilistic oracle (simulates quantum-like behavior)
        """
        from itertools import combinations
        import random
        
        results = []
        evaluations = 0
        
        # Random sampling with bias
        all_subsets = []
        for r in range(1, min(8, self.n_elements + 1)):  # Limit subset size
            all_subsets.extend(combinations(range(self.n_elements), r))
        
        # Sample subsets randomly
        random.shuffle(all_subsets)
        
        for subset_indices in all_subsets[:max_evaluations]:
            if evaluations >= max_evaluations:
                break
                
            evaluations += 1
            
            # Use probabilistic oracle
            if self.probabilistic_evaluate(subset_indices):
                # Verify with true oracle
                if self.evaluate(subset_indices):
                    details = self.evaluate_with_details(subset_indices)
                    results.append(details)
        
        return results, evaluations

# Run comprehensive demonstration
if __name__ == "__main__":
    print(f"\n{'='*60}")
    print("PROBABILISTIC ORACLE DEMONSTRATION")
    print("="*60)
    
    # Test probabilistic oracle
    integer_set = np.random.randint(1, 1000, size=40)
    target = 555
    
    prob_oracle = ProbabilisticOracle(integer_set, target, bias_strength=0.3)
    solutions, evaluations = prob_oracle.biased_search(max_evaluations=50000000000)
    
    print(f"Probabilistic search on {integer_set} for target {target}:")
    print(f"Found {len(solutions)} solutions in {evaluations} evaluations")
    
    for i, sol in enumerate(solutions[:5], 1):
        print(f"  {i}. {sol['values']} = {sol['sum']}")
