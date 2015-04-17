class WeightUpdater :IterationCallback{
  WeightUpdater(std::shared_ptr<std::vector<)
  ~WeightUpdater() {}
  CallbackReturnType operator()(const IterationSummary& summary) = 0;
}