Shared properties:

All the feature-rendering drivers share the following properties (in addition
to those above):

    :styles:                Stylesheet to use to render features (see: :doc:`/references/symbology`)
    :layout:                Paged data layout (see: :doc:`/user/features`)
    :cache_policy:          Caching policy (see: :doc:`/user/caching`)
    :fading:                Fading behavior (see: Fading_)
    :feature_name:          Expression evaluating to the attribute name containing the feature name
    :feature_indexing:      Whether to index features for query (default is ``false``)
    :lighting:              Whether to override and set the lighting mode on this layer (t/f)
    :max_granularity:       Anglular threshold at which to subdivide lines on a globe (degrees)
    :shader_policy:         Options for shader generation (see: `Shader Policy`_)
