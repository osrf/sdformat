/*
 * Copyright 2019 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <string>
#include <gtest/gtest.h>
#include "sdf/sdf.hh"
#include "sdf/Pbr.hh"

#include "test_config.h"

//////////////////////////////////////////////////
TEST(Material, PbrDOM)
{
  const std::string testFile =
    sdf::testing::TestFile("sdf", "material_pbr.sdf");

  // Load the SDF file into DOM
  sdf::Root root;
  EXPECT_TRUE(root.Load(testFile).empty());

  // Get the first model
  const sdf::Model *model = root.ModelByIndex(0);
  ASSERT_NE(nullptr, model);

  // Get the first link
  const sdf::Link *link = model->LinkByIndex(0);
  ASSERT_NE(nullptr, link);
  EXPECT_EQ("link", link->Name());

  // checkt visual materials
  EXPECT_EQ(3u, link->VisualCount());

  // visual metal workflow
  {
    EXPECT_TRUE(link->VisualNameExists("visual_metal_workflow"));
    const sdf::Visual *visual = link->VisualByIndex(0);
    ASSERT_NE(nullptr, visual);
    EXPECT_EQ("visual_metal_workflow", visual->Name());

    // material
    const sdf::Material *material = visual->Material();
    ASSERT_NE(nullptr, material);

    // diffuse
    EXPECT_EQ(ignition::math::Color(0.2f, 0.5f, 0.1f, 1.0f),
        material->Diffuse());

    // pbr
    const sdf::Pbr *pbr = material->PbrMaterial();
    ASSERT_NE(nullptr, pbr);

    // metal
    const sdf::PbrWorkflow *workflow =
        pbr->Workflow(sdf::PbrWorkflowType::METAL);
    ASSERT_NE(nullptr, workflow);
    EXPECT_EQ(sdf::PbrWorkflowType::METAL, workflow->Type());
    EXPECT_EQ(nullptr, pbr->Workflow(sdf::PbrWorkflowType::SPECULAR));

    // albedo map
    EXPECT_EQ("albedo_map.png", workflow->AlbedoMap());

    // normal map
    EXPECT_EQ("normal_map.png", workflow->NormalMap());

    // metalness map
    EXPECT_EQ("metalness_map.png", workflow->MetalnessMap());

    // metalness
    EXPECT_DOUBLE_EQ(0.3, workflow->Metalness());

    // roughness map
    EXPECT_EQ("roughness_map.png", workflow->RoughnessMap());

    // roughness
    EXPECT_DOUBLE_EQ(0.4, workflow->Roughness());

    // environment map
    EXPECT_EQ("environment_map.png", workflow->EnvironmentMap());

    // ambient occlusion map
    EXPECT_EQ("ambient_occlusion_map.png", workflow->AmbientOcclusionMap());

    // emissive map
    EXPECT_EQ("emissive_map.png", workflow->EmissiveMap());

    // light map
    EXPECT_EQ("light_map.png", workflow->LightMap());
    EXPECT_EQ(1u, workflow->LightMapTexCoordSet());
  }

  // visual specular workflow
  {
    EXPECT_TRUE(link->VisualNameExists("visual_specular_workflow"));
    const sdf::Visual *visual = link->VisualByIndex(1);
    ASSERT_NE(nullptr, visual);
    EXPECT_EQ("visual_specular_workflow", visual->Name());

    // material
    const sdf::Material *material = visual->Material();
    ASSERT_NE(nullptr, material);

    // diffuse
    EXPECT_EQ(ignition::math::Color(0.2f, 0.5f, 0.1f, 1.0f),
        material->Diffuse());

    // pbr
    const sdf::Pbr *pbr = material->PbrMaterial();
    ASSERT_NE(nullptr, pbr);

    // specular
    const sdf::PbrWorkflow *workflow =
        pbr->Workflow(sdf::PbrWorkflowType::SPECULAR);
    ASSERT_NE(nullptr, workflow);
    EXPECT_EQ(sdf::PbrWorkflowType::SPECULAR, workflow->Type());
    EXPECT_EQ(nullptr, pbr->Workflow(sdf::PbrWorkflowType::METAL));

    // albedo map
    EXPECT_EQ("albedo_map.png", workflow->AlbedoMap());

    // normal map
    EXPECT_EQ("normal_map.png", workflow->NormalMap());

    // metalness map
    EXPECT_EQ("glossiness_map.png", workflow->GlossinessMap());

    // glossiness
    EXPECT_DOUBLE_EQ(0.2, workflow->Glossiness());

    // specular map
    EXPECT_EQ("specular_map.png", workflow->SpecularMap());

    // environment map
    EXPECT_EQ("environment_map.png", workflow->EnvironmentMap());

    // ambient occlusion map
    EXPECT_EQ("ambient_occlusion_map.png", workflow->AmbientOcclusionMap());

    // emissive map
    EXPECT_EQ("emissive_map.png", workflow->EmissiveMap());

    // light map
    EXPECT_EQ("light_map.png", workflow->LightMap());
    EXPECT_EQ(2u, workflow->LightMapTexCoordSet());
  }

  // visual all
  {
    EXPECT_TRUE(link->VisualNameExists("visual_all"));
    const sdf::Visual *visual = link->VisualByIndex(2);
    ASSERT_NE(nullptr, visual);
    EXPECT_EQ("visual_all", visual->Name());

    // material
    const sdf::Material *material = visual->Material();
    ASSERT_NE(nullptr, material);

    // diffuse
    EXPECT_EQ(ignition::math::Color(0.2f, 0.5f, 0.1f, 1.0f),
        material->Diffuse());

    // pbr
    const sdf::Pbr *pbr = material->PbrMaterial();
    ASSERT_NE(nullptr, pbr);

    // specular
    {
      // specular
      const sdf::PbrWorkflow *workflow =
          pbr->Workflow(sdf::PbrWorkflowType::SPECULAR);
      ASSERT_NE(nullptr, workflow);
      EXPECT_EQ(sdf::PbrWorkflowType::SPECULAR, workflow->Type());

      // specular
      EXPECT_EQ(sdf::PbrWorkflowType::SPECULAR, workflow->Type());

      // albedo map
      EXPECT_EQ("albedo_map.png", workflow->AlbedoMap());

      // normal map
      EXPECT_EQ("normal_map_tangent.png", workflow->NormalMap());

      // metalness map
      EXPECT_EQ("glossiness_map.png", workflow->GlossinessMap());

      // glossiness
      EXPECT_DOUBLE_EQ(0.2, workflow->Glossiness());

      // specular map
      EXPECT_EQ("specular_map.png", workflow->SpecularMap());

      // environment map
      EXPECT_EQ("environment_map.png", workflow->EnvironmentMap());

      // ambient occlusion map
      EXPECT_EQ("ambient_occlusion_map.png", workflow->AmbientOcclusionMap());

      // emissive map
      EXPECT_EQ("emissive_map.png", workflow->EmissiveMap());

      // light map
      EXPECT_EQ("light_map.png", workflow->LightMap());
      EXPECT_EQ(3u, workflow->LightMapTexCoordSet());
    }
    // metal
    {
      const sdf::PbrWorkflow *workflow =
          pbr->Workflow(sdf::PbrWorkflowType::METAL);
      ASSERT_NE(nullptr, workflow);
      EXPECT_EQ(sdf::PbrWorkflowType::METAL, workflow->Type());

      // albedo map
      EXPECT_EQ("albedo_map.png", workflow->AlbedoMap());

      // normal map
      EXPECT_EQ("normal_map_object.png", workflow->NormalMap());

      // metalness map
      EXPECT_EQ("metalness_map.png", workflow->MetalnessMap());

      // metalness
      EXPECT_DOUBLE_EQ(0.3, workflow->Metalness());

      // roughness map
      EXPECT_EQ("roughness_map.png", workflow->RoughnessMap());

      // roughness
      EXPECT_DOUBLE_EQ(0.4, workflow->Roughness());

      // environment map
      EXPECT_EQ("environment_map.png", workflow->EnvironmentMap());

      // ambient occlusion map
      EXPECT_EQ("ambient_occlusion_map.png", workflow->AmbientOcclusionMap());

      // emissive map
      EXPECT_EQ("emissive_map.png", workflow->EmissiveMap());

      // light map
      EXPECT_EQ("light_map.png", workflow->LightMap());
      EXPECT_EQ(0u, workflow->LightMapTexCoordSet());
    }
  }
}

//////////////////////////////////////////////////
TEST(Material, MaterialPBR)
{
  const std::string testFile =
    sdf::testing::TestFile("sdf", "material_pbr.sdf");

  // load material pbr sdf file
  sdf::Errors errors;
  sdf::SDFPtr sdfParsed = sdf::readFile(testFile, errors);
  ASSERT_TRUE(errors.empty());
  ASSERT_NE(nullptr, sdfParsed);
  ASSERT_NE(nullptr, sdfParsed->Root());
  EXPECT_EQ(testFile, sdfParsed->FilePath());

  // model
  EXPECT_TRUE(sdfParsed->Root()->HasElement("model"));
  sdf::ElementPtr modelElem = sdfParsed->Root()->GetElement("model");
  EXPECT_TRUE(modelElem->HasAttribute("name"));
  EXPECT_EQ(modelElem->Get<std::string>("name"), "model");
  EXPECT_EQ(testFile, modelElem->FilePath());

  // link
  EXPECT_TRUE(modelElem->HasElement("link"));
  sdf::ElementPtr linkElem = modelElem->GetElement("link");
  EXPECT_TRUE(linkElem->HasAttribute("name"));
  EXPECT_EQ(linkElem->Get<std::string>("name"), "link");
  EXPECT_EQ(testFile, linkElem->FilePath());

  EXPECT_TRUE(linkElem->HasElement("visual"));
  sdf::ElementPtr visualElem = linkElem->GetElement("visual");

  // visual metal workflow
  {
    EXPECT_TRUE(visualElem->HasAttribute("name"));
    EXPECT_EQ("visual_metal_workflow", visualElem->Get<std::string>("name"));
    EXPECT_EQ(testFile, visualElem->FilePath());

    // material
    EXPECT_TRUE(visualElem->HasElement("material"));
    sdf::ElementPtr materialElem = visualElem->GetElement("material");

    // diffuse
    EXPECT_TRUE(materialElem->HasElement("diffuse"));
    sdf::ElementPtr diffuseElem = materialElem->GetElement("diffuse");
    EXPECT_EQ(ignition::math::Color(0.2f, 0.5f, 0.1f, 1.0f),
        diffuseElem->Get<ignition::math::Color>());

    // pbr
    EXPECT_TRUE(materialElem->HasElement("pbr"));
    sdf::ElementPtr pbrElem = materialElem->GetElement("pbr");

    // metal
    EXPECT_TRUE(pbrElem->HasElement("metal"));
    sdf::ElementPtr metalElem = pbrElem->GetElement("metal");

    // albedo map
    EXPECT_TRUE(metalElem->HasElement("albedo_map"));
    sdf::ElementPtr albedoMapElem = metalElem->GetElement("albedo_map");
    EXPECT_EQ("albedo_map.png", albedoMapElem->Get<std::string>());

    // normal map
    EXPECT_TRUE(metalElem->HasElement("normal_map"));
    sdf::ElementPtr normalMapElem = metalElem->GetElement("normal_map");
    EXPECT_TRUE(normalMapElem->HasAttribute("type"));
    EXPECT_EQ("tangent", normalMapElem->Get<std::string>("type"));
    EXPECT_EQ("normal_map.png", normalMapElem->Get<std::string>());

    // metalness map
    EXPECT_TRUE(metalElem->HasElement("metalness_map"));
    sdf::ElementPtr metalnessMapElem = metalElem->GetElement("metalness_map");
    EXPECT_EQ("metalness_map.png", metalnessMapElem->Get<std::string>());

    // metalness
    EXPECT_TRUE(metalElem->HasElement("metalness"));
    sdf::ElementPtr metalnessElem = metalElem->GetElement("metalness");
    EXPECT_DOUBLE_EQ(0.3, metalnessElem->Get<double>());

    // roughness map
    EXPECT_TRUE(metalElem->HasElement("roughness_map"));
    sdf::ElementPtr roughnessMapElem = metalElem->GetElement("roughness_map");
    EXPECT_EQ("roughness_map.png", roughnessMapElem->Get<std::string>());

    // roughness
    EXPECT_TRUE(metalElem->HasElement("roughness"));
    sdf::ElementPtr roughnessElem = metalElem->GetElement("roughness");
    EXPECT_DOUBLE_EQ(0.4, roughnessElem->Get<double>());

    // environment map
    EXPECT_TRUE(metalElem->HasElement("environment_map"));
    sdf::ElementPtr envMapElem = metalElem->GetElement("environment_map");
    EXPECT_EQ("environment_map.png", envMapElem->Get<std::string>());

    // ambient occlusion map
    EXPECT_TRUE(metalElem->HasElement("ambient_occlusion_map"));
    sdf::ElementPtr aoMapElem = metalElem->GetElement("ambient_occlusion_map");
    EXPECT_EQ("ambient_occlusion_map.png", aoMapElem->Get<std::string>());

    // emissive map
    EXPECT_TRUE(metalElem->HasElement("emissive_map"));
    sdf::ElementPtr emissiveMapElem = metalElem->GetElement("emissive_map");
    EXPECT_EQ("emissive_map.png", emissiveMapElem->Get<std::string>());


    // light map
    EXPECT_TRUE(metalElem->HasElement("light_map"));
    sdf::ElementPtr lightMapElem = metalElem->GetElement("light_map");
    EXPECT_EQ("light_map.png", lightMapElem->Get<std::string>());
    EXPECT_EQ(1u, lightMapElem->Get<unsigned int>("uv_set"));
  }

  // visual specular workflow
  {
    visualElem = visualElem->GetNextElement("visual");
    EXPECT_TRUE(visualElem->HasAttribute("name"));
    EXPECT_EQ("visual_specular_workflow", visualElem->Get<std::string>("name"));

    // material
    EXPECT_TRUE(visualElem->HasElement("material"));
    sdf::ElementPtr materialElem = visualElem->GetElement("material");

    // diffuse
    EXPECT_TRUE(materialElem->HasElement("diffuse"));
    sdf::ElementPtr diffuseElem = materialElem->GetElement("diffuse");
    EXPECT_EQ(ignition::math::Color(0.2f, 0.5f, 0.1f, 1.0f),
        diffuseElem->Get<ignition::math::Color>());

    // pbr
    EXPECT_TRUE(materialElem->HasElement("pbr"));
    sdf::ElementPtr pbrElem = materialElem->GetElement("pbr");

    // specular
    EXPECT_TRUE(pbrElem->HasElement("specular"));
    sdf::ElementPtr specularElem = pbrElem->GetElement("specular");

    // albedo map
    EXPECT_TRUE(specularElem->HasElement("albedo_map"));
    sdf::ElementPtr albedoMapElem = specularElem->GetElement("albedo_map");
    EXPECT_EQ("albedo_map.png", albedoMapElem->Get<std::string>());

    // normal map
    EXPECT_TRUE(specularElem->HasElement("normal_map"));
    sdf::ElementPtr normalMapElem = specularElem->GetElement("normal_map");
    EXPECT_TRUE(normalMapElem->HasAttribute("type"));
    EXPECT_EQ("tangent", normalMapElem->Get<std::string>("type"));
    EXPECT_EQ("normal_map.png", normalMapElem->Get<std::string>());

    // specular map
    EXPECT_TRUE(specularElem->HasElement("specular_map"));
    sdf::ElementPtr specularMapElem = specularElem->GetElement("specular_map");
    EXPECT_EQ("specular_map.png", specularMapElem->Get<std::string>());

    // glossiness map
    EXPECT_TRUE(specularElem->HasElement("glossiness_map"));
    sdf::ElementPtr glossinessMapElem =
        specularElem->GetElement("glossiness_map");
    EXPECT_EQ("glossiness_map.png", glossinessMapElem->Get<std::string>());

    // glossiness
    EXPECT_TRUE(specularElem->HasElement("glossiness"));
    sdf::ElementPtr glossinessElem = specularElem->GetElement("glossiness");
    EXPECT_DOUBLE_EQ(0.2, glossinessElem->Get<double>());

    // environment map
    EXPECT_TRUE(specularElem->HasElement("environment_map"));
    sdf::ElementPtr envMapElem = specularElem->GetElement("environment_map");
    EXPECT_EQ("environment_map.png", envMapElem->Get<std::string>());

    // ambient occlusion map
    EXPECT_TRUE(specularElem->HasElement("ambient_occlusion_map"));
    sdf::ElementPtr aoMapElem =
        specularElem->GetElement("ambient_occlusion_map");
    EXPECT_EQ("ambient_occlusion_map.png", aoMapElem->Get<std::string>());

    // emissive map
    EXPECT_TRUE(specularElem->HasElement("emissive_map"));
    sdf::ElementPtr emissiveMapElem = specularElem->GetElement("emissive_map");
    EXPECT_EQ("emissive_map.png", emissiveMapElem->Get<std::string>());

    // light map
    EXPECT_TRUE(specularElem->HasElement("light_map"));
    sdf::ElementPtr lightMapElem = specularElem->GetElement("light_map");
    EXPECT_EQ("light_map.png", lightMapElem->Get<std::string>());
    EXPECT_EQ(2u, lightMapElem->Get<unsigned int>("uv_set"));
  }

  // visual all
  {
    visualElem = visualElem->GetNextElement("visual");
    EXPECT_TRUE(visualElem->HasAttribute("name"));
    EXPECT_EQ("visual_all", visualElem->Get<std::string>("name"));

    // material
    EXPECT_TRUE(visualElem->HasElement("material"));
    sdf::ElementPtr materialElem = visualElem->GetElement("material");

    // diffuse
    EXPECT_TRUE(materialElem->HasElement("diffuse"));
    sdf::ElementPtr diffuseElem = materialElem->GetElement("diffuse");
    EXPECT_EQ(ignition::math::Color(0.2f, 0.5f, 0.1f, 1.0f),
        diffuseElem->Get<ignition::math::Color>());

    // pbr
    EXPECT_TRUE(materialElem->HasElement("pbr"));
    sdf::ElementPtr pbrElem = materialElem->GetElement("pbr");

    {
      // specular
      EXPECT_TRUE(pbrElem->HasElement("specular"));
      sdf::ElementPtr specularElem = pbrElem->GetElement("specular");

      // albedo map
      EXPECT_TRUE(specularElem->HasElement("albedo_map"));
      sdf::ElementPtr albedoMapElem = specularElem->GetElement("albedo_map");
      EXPECT_EQ("albedo_map.png", albedoMapElem->Get<std::string>());

      // normal map
      EXPECT_TRUE(specularElem->HasElement("normal_map"));
      sdf::ElementPtr normalMapElem = specularElem->GetElement("normal_map");
      EXPECT_TRUE(normalMapElem->HasAttribute("type"));
      EXPECT_EQ("tangent", normalMapElem->Get<std::string>("type"));
      EXPECT_EQ("normal_map_tangent.png", normalMapElem->Get<std::string>());

      // specular map
      EXPECT_TRUE(specularElem->HasElement("specular_map"));
      sdf::ElementPtr specularMapElem =
          specularElem->GetElement("specular_map");
      EXPECT_EQ("specular_map.png", specularMapElem->Get<std::string>());

      // glossiness map
      EXPECT_TRUE(specularElem->HasElement("glossiness_map"));
      sdf::ElementPtr glossinessMapElem =
          specularElem->GetElement("glossiness_map");
      EXPECT_EQ("glossiness_map.png", glossinessMapElem->Get<std::string>());

      // glossiness
      EXPECT_TRUE(specularElem->HasElement("glossiness"));
      sdf::ElementPtr glossinessElem = specularElem->GetElement("glossiness");
      EXPECT_DOUBLE_EQ(0.2, glossinessElem->Get<double>());

      // environment map
      EXPECT_TRUE(specularElem->HasElement("environment_map"));
      sdf::ElementPtr envMapElem = specularElem->GetElement("environment_map");
      EXPECT_EQ("environment_map.png", envMapElem->Get<std::string>());

      // ambient occlusion map
      EXPECT_TRUE(specularElem->HasElement("ambient_occlusion_map"));
      sdf::ElementPtr aoMapElem =
          specularElem->GetElement("ambient_occlusion_map");
      EXPECT_EQ("ambient_occlusion_map.png", aoMapElem->Get<std::string>());

      // emissive map
      EXPECT_TRUE(specularElem->HasElement("emissive_map"));
      sdf::ElementPtr emissiveMapElem =
          specularElem->GetElement("emissive_map");
      EXPECT_EQ("emissive_map.png", emissiveMapElem->Get<std::string>());

      // light map
      EXPECT_TRUE(specularElem->HasElement("light_map"));
      sdf::ElementPtr lightMapElem = specularElem->GetElement("light_map");
      EXPECT_EQ("light_map.png", lightMapElem->Get<std::string>());
      EXPECT_EQ(3u, lightMapElem->Get<unsigned int>("uv_set"));
    }

    {
      // metal
      EXPECT_TRUE(pbrElem->HasElement("metal"));
      sdf::ElementPtr metalElem = pbrElem->GetElement("metal");

      // albedo map
      EXPECT_TRUE(metalElem->HasElement("albedo_map"));
      sdf::ElementPtr albedoMapElem = metalElem->GetElement("albedo_map");
      EXPECT_EQ("albedo_map.png", albedoMapElem->Get<std::string>());

      // normal map
      EXPECT_TRUE(metalElem->HasElement("normal_map"));
      sdf::ElementPtr normalMapElem = metalElem->GetElement("normal_map");
      EXPECT_TRUE(normalMapElem->HasAttribute("type"));
      EXPECT_EQ("object", normalMapElem->Get<std::string>("type"));
      EXPECT_EQ("normal_map_object.png", normalMapElem->Get<std::string>());

      // metalness map
      EXPECT_TRUE(metalElem->HasElement("metalness_map"));
      sdf::ElementPtr metalnessMapElem = metalElem->GetElement("metalness_map");
      EXPECT_EQ("metalness_map.png", metalnessMapElem->Get<std::string>());

      // metalness
      EXPECT_TRUE(metalElem->HasElement("metalness"));
      sdf::ElementPtr metalnessElem = metalElem->GetElement("metalness");
      EXPECT_DOUBLE_EQ(0.3, metalnessElem->Get<double>());

      // roughness map
      EXPECT_TRUE(metalElem->HasElement("roughness_map"));
      sdf::ElementPtr roughnessMapElem = metalElem->GetElement("roughness_map");
      EXPECT_EQ("roughness_map.png", roughnessMapElem->Get<std::string>());

      // roughness
      EXPECT_TRUE(metalElem->HasElement("roughness"));
      sdf::ElementPtr roughnessElem = metalElem->GetElement("roughness");
      EXPECT_DOUBLE_EQ(0.4, roughnessElem->Get<double>());

      // environment map
      EXPECT_TRUE(metalElem->HasElement("environment_map"));
      sdf::ElementPtr envMapElem = metalElem->GetElement("environment_map");
      EXPECT_EQ("environment_map.png", envMapElem->Get<std::string>());

      // ambient occlusion map
      EXPECT_TRUE(metalElem->HasElement("ambient_occlusion_map"));
      sdf::ElementPtr aoMapElem =
          metalElem->GetElement("ambient_occlusion_map");
      EXPECT_EQ("ambient_occlusion_map.png", aoMapElem->Get<std::string>());

      // emissive map
      EXPECT_TRUE(metalElem->HasElement("emissive_map"));
      sdf::ElementPtr emissiveMapElem = metalElem->GetElement("emissive_map");
      EXPECT_EQ("emissive_map.png", emissiveMapElem->Get<std::string>());

      // light map
      EXPECT_TRUE(metalElem->HasElement("light_map"));
      sdf::ElementPtr lightMapElem = metalElem->GetElement("light_map");
      EXPECT_EQ("light_map.png", lightMapElem->Get<std::string>());
      EXPECT_EQ(0u, lightMapElem->Get<unsigned int>("uv_set"));
    }
  }
}
