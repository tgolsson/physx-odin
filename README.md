<div align="center">

# 🎳 physx-odin

**Unsafe automatically-generated Odin bindings for [NVIDIA PhysX 5.1](https://github.com/NVIDIA-Omniverse/PhysX) C++ API.**

</div>

This is a fork from [`physx-rs`](https://github.com/EmbarkStudios/physx-rs), using the same methodology for binding
generation. Check out the [`physx-sys`](https://github.com/EmbarkStudios/physx-rs/tree/main/physx-sys) and related
presentation for more information.

## Getting started


As of right now, no libraries are shipped as they'd be > 200 MB in
total. Soon~ I hope to have prebuilt binaries + generated files as
Github releases. However, building everything locally *should* be
easy: the provided `justfile` contains a build command. This should
run all codegen, followed by building the native libraries and copying
them to the repository root.

## Basic usage


```c
main :: proc() {
	using physx

	foundation := create_foundation_ext()
	dispatcher := default_cpu_dispatcher_create(
		1,
		nil,
		physx.DefaultCpuDispatcherWaitForWorkMode.WaitForWork,
		0,
	)

	physics := create_physics_ext(foundation)


scene_desc := scene_desc_new(tolerances_scale_new(1.0, 10.0))
	scene_desc.gravity = vec3_new_3(0.0, -9.81, 0.0)
	scene_desc.cpuDispatcher = dispatcher
	scene_desc.filterShader = get_default_simulation_filter_shader()
	scene := physics_create_scene_mut(physics, scene_desc)
	defer {
		scene_release_mut(scene)
		physics_release_mut(physics)
		default_cpu_dispatcher_release_mut(dispatcher)
		foundation_release_mut(foundation)
	}

	run(physics, scene)
}
```

## Examples

### [Ball](examples/ball.rs)

A simple example to showcase how to use physx-sys. It can be run with `odin run examples/ball.odin -file`.

```txt
Simulated 100 timesteps in 8.161109 ms.
Heights over time (x=time, dt=100ms)

                  o

                   o

                    o                   ooooooo
                                      oo       ooo
                     o               o            o
                                   oo              o
                                  o                 o
                      o          o                   o
                                o                     o
                       o       o                       o
                                                        o           ooooooo
                        o     o                          o        oo       oo
                             o                                  oo           oo
                            o                             o    o               o    ooooooooo
                         o o                               o oo                 oooo         ooooooo
```

## License

Licensed under either of

* Apache License, Version 2.0, ([LICENSE-APACHE](LICENSE-APACHE) or <http://www.apache.org/licenses/LICENSE-2.0>)
* MIT license ([LICENSE-MIT](LICENSE-MIT) or <http://opensource.org/licenses/MIT>)

at your option.

Note that the [PhysX C++ SDK](https://github.com/NVIDIA-Omniverse/PhysX) has its [own BSD 3 license](LICENSE-BSD).

### Contribution

Unless you explicitly state otherwise, any contribution intentionally
submitted for inclusion in the work by you, as defined in the
Apache-2.0 license, shall be dual licensed as above, without any
additional terms or conditions.
