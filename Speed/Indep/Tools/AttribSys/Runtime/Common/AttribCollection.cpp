#include "Speed/indep/Tools/AttribSys/Runtime/AttribCollection.h"
#include "Speed/indep/Tools/AttribSys/Runtime/AttribClass.h"
#include "Speed/indep/Tools/AttribSys/Runtime/AttribNode.h"


// MATCHING
Attrib::Node* Attrib::Collection::GetNode(uint32_t attributeKey, const Attrib::Collection*& container)
{
	Collection* c = this;
	// climb the node tree
	while (c)
	{
		Node* node = c->mTable.Find(attributeKey);
		// node has been found
		if (node)
		{
			container = c;
			return node;
		}
		else
		{
			c = (Collection *)c->mParent;
		}
	}
	
	if (mLayout)
	{
		Node* node = mClass->GetPrivates().GetLayoutTable().Find(attributeKey);
		if (node)
		{
			container = this;
			return node;
		}		
	}
	container = NULL;
	return NULL;
}

// MATCHING
void* Attrib::Collection::GetData(uint32_t attributeKey, uint32_t index)
{
	const Collection* c;
	Node* node = GetNode(attributeKey, c);
	if (node)
	{
		if (node->IsArray())  return node->GetArray(c->mLayout)->GetData(index);
		else  return node->GetPointer(c->mLayout);
	}
	else
	{
		return NULL;
	}
}